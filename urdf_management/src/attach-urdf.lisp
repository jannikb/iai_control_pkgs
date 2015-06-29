;;; Copyright (c) 2014, Jannik Buckelo <jannikbu@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :urdf-management)

(defgeneric add-prefix (object prefix)
  (:documentation "Adds the `prefix' in front of the name of the `object'."))

(defgeneric make-link-root (robot-model link)
  (:documentation "Makes the `link' the new root link of the `robot'."))

(defun read-urdf-xml (urdf-path)
  "Gets a path to an urdf that can either be an absolute path or a ros path
 and returns a xml-struct of it."
  (s-xml:parse-xml-file (get-absolute-path urdf-path) :output-type :xml-struct))

(defun get-absolute-path (path)
  (if (eql (first (pathname-directory path)) :absolute)
      path
      (ros-path->absolute-path path)))
  
(defun ros-path->absolute-path (ros-path)
  "Converts a ros-path of the form '<ros-pkg-name>/rest/of/the.path' to an absolute path."
  (let* ((pkg-name (second (pathname-directory ros-path)))
         (absolute-pkg-path (ros-load-manifest:ros-package-path pkg-name))
         (path-from-pkg (subseq (namestring ros-path) 
                                (1+ (length pkg-name)))))
    (pathname (format nil "~a~a" (namestring absolute-pkg-path) path-from-pkg))))

(defmethod add-prefix ((robot-model robot) prefix)
  (flet ((add-prefix-helper (key value)
           (declare (ignore key))
           (add-prefix value prefix)))
    (maphash #'add-prefix-helper (links robot-model))
    (maphash #'add-prefix-helper (joints robot-model))))

(defmethod add-prefix ((joint joint) prefix)
  (setf (name joint)
        (add-prefix (name joint) prefix))
  (setf (parent-name joint)
        (add-prefix (parent-name joint) prefix))
  (setf (child-name joint)
        (add-prefix (child-name joint) prefix)))

(defmethod add-prefix ((link link) prefix)
  (setf (name link)
        (add-prefix (name link) prefix)))

(defmethod add-prefix ((name string) prefix)
  (format nil "~a~a" prefix name))

(defmethod make-link-root ((robot-model robot) (link-name string))
  "Makes the link of the `robot-model' with the name `link-name' the root link of the robot."
  (let ((link (gethash link-name (links robot-model))))
    (unless link
      (ros-warn (urdf-management) "Link ~a not found." link-name)
      (return-from make-link-root nil))
    (make-link-root robot-model link)))

(defmethod make-link-root ((robot-model robot) (link link))
  "Makes the `link' the root link of the `robot-model'. `link' has to be a part of `robot-model'."
  (when (from-joint link)
    (let ((links-to-root nil)
          (current-link link))
      ;; get the links from the new root link to the old one
      (loop while (from-joint current-link)
            do (push current-link links-to-root)
               (setf current-link (parent (from-joint current-link))))
      (mapcar #'make-temp-root links-to-root)
      (setf (root-link robot-model) link)))
  robot-model)

(defun make-temp-root (link)
  "Works under the implication that the parent of `link' is the root link"
  (let* ((joint (from-joint link))
         (old-root (parent joint)))
    (setf (from-joint old-root) joint)
    (setf (from-joint link) nil)
    (setf (to-joints old-root)
          (remove joint (to-joints old-root)))
    (setf (to-joints link) (cons joint (to-joints link)))
    (setf (parent joint) link)
    (setf (child joint) old-root)
    (setf (parent-name joint) (name link))
    (setf (child-name joint) (name old-root))
    (setf (origin joint)
          (cl-transforms:transform-inv (origin joint))))
  link)
  
(defun attach-robot! (base-robot robot-to-attach joint &optional prefix)
  "Attaches the `robot-to-attach' to the `base-robot'. The `joint' is used to connect those to robots and if necessary the child of the joint will be made the root link of `robot-to-attach'. Optionally the `prefix' will be added to all the joints and links of the `robot-to-attach'."
  (flet ((add-link-helper (key link)
           (declare (ignore key))
           (setf (gethash (if prefix
                              (add-prefix (name link) prefix)
                              (name link))
                          (links base-robot)) link))
         (add-joint-helper (key joint)
           (declare (ignore key))
           (setf (gethash (if prefix
                              (add-prefix (name joint) prefix)
                              (name joint))
                          (joints base-robot)) joint)))
    ;; Add the prefix if given
    (when (and prefix (= (length prefix) 0))
      (add-prefix robot-to-attach prefix))
    ;; Make the link that gets connected to the robot root
    (when (make-link-root robot-to-attach (subseq (child-name joint) (length prefix)))
      ;; Add the links and joints to the robot
      (maphash #'add-joint-helper (joints robot-to-attach))
      (maphash #'add-link-helper (links robot-to-attach))
      ;; Set the parent and child fields of the connecting joint
      (add-joint-helper nil joint)
      (setf (parent joint)
            (gethash (parent-name joint) (links base-robot)))
      (setf (child joint)
            (gethash (child-name joint) (links base-robot)))
      base-robot)))

(defun attach-robot (base-robot robot-to-attach joint &optional prefix)
  (attach-robot! (copy-object base-robot) robot-to-attach joint prefix))

