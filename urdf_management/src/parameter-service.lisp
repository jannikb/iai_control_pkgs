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

(def-service-callback SimpleAlterUrdf (action parameter)
  (ros-info (urdf-management) "Altering robot description.")
  (when (eql action (symbol-code 'iai_urdf_msgs-srv:alterurdf-request :add))
    (let ((description (get-param (concatenate 'string "urdf_management/" parameter))))
      (unless description
        ;; error msg
        (make-response :success nil))
      (when description
         (make-response :success (add-description description)))))
  (when (eql action (symbol-code 'iai_urdf_msgs-srv:alterurdf-request :remove))
    (let* ((description (get-param (concatenate 'string "urdf_management/" parameter)))
           (names (when description (get-element-names description))))
      (unless names
        ;; error msg
        (make-response :success nil))
      (when names
        (make-response :success (remove-names names)))))
  ;; error msg
  (make-response :success nil))

(defun simple-alter-urdf-service ()
  "Registers the service to alter the robot description."
  (register-service "simple_alter_urdf" 'SimpleAlterUrdf)
  (ros-info (urdf-management) "Ready to alter urdf."))

(defun add-description (description)
  (call-service "alter_urdf" 'iai_urdf_msgs-srv:alterurdf 
                :action (symbol-code 'iai_urdf_msgs-srv:alterurdf-request :add)
                :xml_elements_to_add description))
    
(defun remove-names (names)
  (call-service "alter_urdf" 'iai_urdf_msgs-srv:alterurdf 
                :action (symbol-code 'iai_urdf_msgs-srv:alterurdf-request :remove)
                :element_names_to_remove names))

(defun get-element-names (description)
  (let ((parsed-xml (s-xml:parse-xml-string (format nil "<container>~a</container>" 
                                                    description)
                                            :output-type :xml-struct)))
    (mapcar (lambda (child)
              (let ((child-type (s-xml:xml-element-name child)))
                (when (or (eql child-type :|link|) (eql child-type :|joint|))
                  (s-xml:xml-element-attribute child :|name|))))
            (s-xml:xml-element-children parsed-xml))))