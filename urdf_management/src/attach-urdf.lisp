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

(defun urdf-to-attach (urdf-path joint-string &optional prefix)
  "Returns a string of urdf description with the description of `joint-string' added 
and all link and joint names have `prefix' added before them."
  (let ((urdf-xml (read-urdf-xml urdf-path))
        (joint-xml (s-xml:parse-xml-string joint-string :output-type :xml-struct)))
    (when prefix
      ;; Add prefixes to the joint that connects the new urdf with the robot
      ;; except the parent of that joint
      (add-prefix-name joint-xml prefix)
      (add-prefix-to-joint-elements joint-xml :|child| prefix)
      ;; Add prefixes to the joints and links of the urdf
      (dolist (child (s-xml:xml-element-children urdf-xml))
        (case (s-xml:xml-element-name child)
          (:|link| (add-prefix-name child prefix))
          (:|joint| (add-prefix-joint child prefix)))))
    ;; Add the xml of the connector joint to the urdf xml
    (push joint-xml (s-xml:xml-element-children urdf-xml))
    (s-xml:print-xml-string urdf-xml :input-type :xml-struct)))

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
                              
(defun add-prefix-name (element prefix)
  (setf (s-xml:xml-element-attribute element :|name|)
        (format nil "~a~a"
                prefix
                (s-xml:xml-element-attribute element :|name|))))

(defun add-prefix-joint (joint-xml prefix)
  (add-prefix-name joint-xml prefix)
  (add-prefix-to-joint-elements joint-xml :|child| prefix)
  (add-prefix-to-joint-elements joint-xml :|parent| prefix))

(defun add-prefix-to-joint-elements (joint-xml key prefix)
  (let ((xml (cl-urdf::xml-element-child joint-xml key)))
    (setf (s-xml:xml-element-attribute xml :|link|)
          (format nil "~a~a" prefix (s-xml:xml-element-attribute xml :|link|)))))

(defun get-urdf-links (urdf-path &optional prefix)
  (let ((links (mapcar (lambda (child) (s-xml:xml-element-attribute child :|name|))
                       (remove-if-not (lambda (child)
                                        (eql (s-xml:xml-element-name child) :|link|))
                                      (s-xml:xml-element-children (read-urdf-xml urdf-path))))))
    (if (and prefix (not (equal prefix "")))
        (mapcar (lambda (link) (concatenate 'string prefix link))
                links)
        links)))
         
