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

(defun start-simple-service ()
  (with-ros-node ("urdf_management" :spin t)
    (simple-alter-urdf-service)))

(def-service-callback SimpleAlterUrdf (action parameter)
  "Callback for the 'simple_alter_urdf' service."
  (ros-info (urdf-management simple-service) "Altering robot description.")
  (cond
    ((eql action (symbol-code 'iai_urdf_msgs-srv:simplealterurdf-request :add))
     (let ((description (get-param (concatenate 'string "urdf_management/" parameter) nil)))
       (if description
           (make-response :success (add-description description))
           (progn
             (ros-error (urdf-management simple-service) "~a not found on parameter server." parameter)
             (make-response :success nil)))))
    ((eql action (symbol-code 'iai_urdf_msgs-srv:simplealterurdf-request :remove))
     (let ((description (get-param (concatenate 'string "urdf_management/" parameter) nil)))
       (if description
           (let ((links (when description (get-link-names description))))
             (make-response :success (remove-links links)))
           (progn    
             (ros-error (urdf-management simple-service) "~a not found on parameter server." parameter)
             (make-response :success nil)))))
    (t (ros-error (urdf-management simple-service) "Action ~a undefined." action)
       (make-response :success nil))))

(defun simple-alter-urdf-service ()
  "Registers the service."
  (register-service "simple_alter_urdf" 'SimpleAlterUrdf)
  (ros-info (urdf-management simple-service) "Ready to alter urdf."))

(defun add-description (description)
  "Calls the AlterUrdf service to add `description' to the robot description."
  (call-alter-urdf (symbol-code 'iai_urdf_msgs-srv:alterurdf-request :add)
                   description nil))
    
(defun remove-links (links)
  "Calls the AlterUrdf service to remove `links' from the robot description."
  (call-alter-urdf (symbol-code 'iai_urdf_msgs-srv:alterurdf-request :remove)
                   "" links))

(defun call-alter-urdf (action add remove &optional (timeout 5))
  "Calls the service alter_urdf with `action', `add' and `remove' as parameters."
  (let ((found-service (wait-for-service "alter_urdf" timeout)))
   (if found-service
       (let ((response (call-service "alter_urdf" 'iai_urdf_msgs-srv:alterurdf 
                               :action action
                               :xml_elements_to_add add
                               :element_names_to_remove remove)))
         (unless (success response)
           (ros-warn (urdf-management simple-service) "AlterUrdf service didn't succeed."))
         (success response))
       (progn
         (ros-error (urdf-management simple-service) "No service 'alter_urdf' found.")
         nil))))
     
(defun get-link-names (description)
  "Gets a xml descritpion of robot parts and returns the names of the links."
  (let ((parsed-xml (s-xml:parse-xml-string (format nil "<container>~a</container>" 
                                                    description)
                                            :output-type :xml-struct)))
    (mapcar (lambda (child) (s-xml:xml-element-attribute child :|name|))
            (remove-if-not (lambda (child) (eql (s-xml:xml-element-name child) :|link|))
                           (s-xml:xml-element-children parsed-xml)))))
