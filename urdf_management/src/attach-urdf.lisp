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
  (let* ((absolute-p (eql (first (pathname-directory urdf-path)) :absolute))
         (urdf-file (if absolute-p
                        urdf-path
                        (ros-path->absolute-path urdf-path))))
    (s-xml:parse-xml-file urdf-file :output-type :xml-struct)))

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
  (make-link-root robot-model (gethash link-name (links robot-model))))

(defmethod make-link-root ((robot-model robot) (link link))
  "Makes the `link' the root link of the `robot-model'. `link' has to be a part of `robot-model'."
  (when (from-joint link)
    (swap-child-with-parent (from-joint link))
    (setf (from-joint link) nil)
    (setf (root-link robot-model) link)))

(defun swap-child-with-parent (child-joint)
  (let* ((link (parent child-joint))
         (parent-joint (from-joint link)))
    (when parent-joint
      (setf (parent parent-joint) link)
      (setf (parent-name parent-joint) (name link))
      (setf (to-joints link)
            (cons parent-joint (to-joints link))))

    (swap-child-with-parent parent-joint)
    (inverse-joint-transformation parent-joint child-joint)
    (setf (child child-joint) link)
    (setf (child-name child-joint) (name link))
    (setf (from-joint link) child-joint)
    (setf (to-joints link)
          (remove child-joint (to-joints link)))))
    
(defun inverse-joint-transformation (parent child)
  "`parent' is the parent joint of `child'. The transform from `parent' will be removed
and `child' gets a transform that is the inverse of it."
  (let* ((rot-mat (matrix->rotation-matrix (cl-transforms:transform->matrix (origin parent))))
         (inv-rot-mat (cl-transforms:invert-rot-matrix rot-mat))
         (inv-trans ))
    (setf (origin child)
          (cl-transforms:make-transform
           inv-trans (cl-transforms:matrix->quaternion inv-rot-mat)))))
          
(defun inverse-translation (translation inv-rot-mat)
  (let ((neg-trans (cl-transforms:make-3d-vector (- (x translation))
                                                 (- (y translation))
                                                 (- (z translation)))))
    (rot-matrix*3d-vector inv-rot-mat
                          neg-trans)))

(defun matrix->rotation-matrix (matrix)
  (make-array '(3 3)
              :initial-contents (loop for y from 0 below 3
                                      collecting (loop for x from 0 below 3
                                                       collecting (aref matrix y x)))))

(defun rot-matrix*3d-vector (matrix v)
  (cl-transforms:make-3d-vector (+ (* (aref matrix 0 0) (x v))
                                   (* (aref matrix 0 1) (x v))
                                   (* (aref matrix 0 2) (x v)))
                                (+ (* (aref matrix 1 0) (y v))
                                   (* (aref matrix 1 1) (y v))
                                   (* (aref matrix 1 2) (y v)))
                                (+ (* (aref matrix 2 0) (z v))
                                   (* (aref matrix 2 1) (z v))
                                   (* (aref matrix 2 2) (z v)))))

(defun x (v)
  (cl-transforms:x v))

(defun y (v)
  (cl-transforms:y v))

(defun z (v)
  (cl-transforms:z v))
  
(defun attach-robot (base-robot robot-to-attach joint &optional prefix)
  "Attaches the `robot-to-attach' to the `base-robot'. The `joint' is used to connect those to robots and if necessary the child of the joint will be made the root link of `robot-to-attach'. Optionally the `prefix' will be added to all the joints and links of the `robot-to-attach'."
  (flet ((add-link-helper (key link)
           (declare (ignore key))
           (setf (gethash (name link) (links base-robot)) link))
         (add-joint-helper (key joint)
           (declare (ignore key))
           (setf (gethash (name joint) (joints base-robot)) joint)))
    ;; Add the prefix if given
    (when prefix
      (add-prefix robot-to-attach prefix))
    ;; Make the link that gets connected to the robot root
    (make-link-root robot-to-attach (child-name joint))
    ;; Add the links and joints to the robot
    (maphash #'add-joint-helper (joints robot-to-attach))
    (maphash #'add-link-helper (links robot-to-attach))
    ;; Set the parent and child fields of the connecting joint
    (add-joint-helper 0 joint)
    (setf (parent joint)
          (gethash (parent-name joint) (links base-robot)))
    (setf (child joint)
          (gethash (child-name joint) (links base-robot))))
  base-robot)



(defun urdf-to-attach (urdf-path joint-string &optional prefix)
  "Returns a string of urdf description with the description of `joint-string' added 
and all link and joint names have `prefix' added before them.")
  ;; (let ((urdf-xml (read-urdf-xml urdf-path))
    ;;     (joint-xml (s-xml:parse-xml-string joint-string :output-type :xml-struct)))
    ;; (when prefix
    ;;   ;; Add prefixes to the joint that connects the new urdf with the robot
    ;;   ;; except the parent of that joint
    ;;   (add-prefix-name joint-xml prefix)
    ;;   (add-prefix-to-joint-elements joint-xml :|child| prefix)
    ;;   ;; Add prefixes to the joints and links of the urdf
    ;;   (dolist (child (s-xml:xml-element-children urdf-xml))
    ;;     (case (s-xml:xml-element-name child)
    ;;       (:|link| (add-prefix-name child prefix))
    ;;       (:|joint| (add-prefix-joint child prefix)))))
    ;; ;; Make the link that should be connected root
    ;; (make-link-root (s-xml:xml-element-attribute
    ;;                  (cl-urdf::xml-element-child joint-xml :|child|) :|link|)
    ;;                 urdf-xml)
    ;; ;; Add the xml of the connector joint to the urdf xml
    ;; (push joint-xml (s-xml:xml-element-children urdf-xml))
    ;; (s-xml:print-xml-string urdf-xml :input-type :xml-struct)))

