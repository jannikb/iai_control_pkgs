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

(defun xml->links-joints (xml-string)
  (let ((parsed-xml (parse-xml-string xml-string))
        (link-descriptions nil)
        (joint-descriptions nil))
    (when parsed-xml
      (dolist (child (s-xml:xml-element-children parsed-xml))
        (case (s-xml:xml-element-name child)
          (:|link| (push child link-descriptions))
          (:|joint| (push child joint-descriptions))
          (otherwise         
           (ros-warn (urdf-management)
                     "Ignoring element: ~a" child))))
      (values (mapcar (lambda (link-desc) (xml-element->link link-desc)) link-descriptions)
              (mapcar (lambda (joint-desc) (xml-element->joint joint-desc)) joint-descriptions)))))

(defun xml->joint (xml-string)
  (let ((xml-struct (parse-xml-string xml-string)))
    (when xml-struct
      (let ((first-child  (s-xml:first-xml-element-child xml-struct)))
        (when (eql (s-xml:xml-element-name first-child) :|joint|)
          (xml-element->joint first-child))))))

(defun xml-element->link (xml-element)
  (cl-urdf::parse-xml-node :|link| xml-element))

(defun xml-element->joint (xml-element)
  (cl-urdf::parse-xml-node :|joint| xml-element))

(defun parse-xml-string (xml)
  (handler-case
      (let* ((parsed (s-xml:parse-xml-string (format nil "<container>~a</container>" xml)
                                             :output-type :xml-struct))
             (first-child (s-xml:first-xml-element-child parsed)))
        (if (and first-child (eql (s-xml:xml-element-name first-child) :|robot|))
            first-child
            parsed))
    (s-xml:xml-parser-error ()
      (prog1 nil
        (ros-error (urdf-management) "Failed to parse: ~a~%" xml)))))
