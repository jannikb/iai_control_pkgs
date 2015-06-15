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

(defgeneric add-link! (robot link joint)
  (:documentation "ad"))

(defmethod add-link! ((robot robot) (link link) (joint joint))
  (unless (and (equal (child-name joint) (name link))
               (not (gethash (name joint) (joints robot)))
               (not (gethash (name link) (links robot)))
               (gethash (parent-name joint) (links robot)))
    (return-from add-link! nil))
  ;; Add link and joint
  (setf (gethash (name link) (links robot)) link)
  (setf (gethash (name joint) (joints robot)) joint)
  ;; Set other properties of the link and joint
  (setf (from-joint link) joint)
  (setf (child joint) link)
  (let ((parent-link (gethash (parent-name joint) (links robot))))
    (setf (parent joint) parent-link)
    (setf (to-joints parent-link) (cons joint (to-joints parent-link))))
  robot)

(defun add-link (robot link joint)
  (add-link! (copy-object robot) (copy-object link) (copy-object joint)))

(defun parse-xml-string (xml)
  (let* ((parsed (s-xml:parse-xml-string (format nil "<container>~a</container>" xml)
                                        :output-type :xml-struct))
         (first-child (s-xml:first-xml-element-child parsed)))
    (if (and first-child (eql (s-xml:xml-element-name first-child) :|robot|))
        first-child
        parsed)))
  

(defun create-link (link-desc robot)
  "Parses the xml description of the link. If it's a valid link the link is returned else nil."
  (let ((link (cl-urdf::parse-xml-node :|link| link-desc robot)))
    link))
