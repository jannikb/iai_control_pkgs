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

(defvar *robot-model* nil)
(defvar *robot-mutex* (make-mutex))
(defvar *urdf-pub* nil)

(defun start-urdf-management ()
  (with-ros-node ("urdf_management/service" :spin t)
    (alter-urdf-service)))

(def-service-callback AlterUrdf (action xml_elements_to_add element_names_to_remove)
  (ros-info (urdf-management) "Altering robot description.")
  (let ((success (callback-handler action xml_elements_to_add 
                                   element_names_to_remove)))
    (when success (publish-urdf))
    (make-response :success success)))

(defun callback-handler (action add remove)
  (cond
    ((eql action (symbol-code 'iai_urdf_msgs-srv:alterurdf-request :add))
     (ros-info (urdf-management) "Adding to robot model")
     (multiple-value-bind (links joints) (xml->links-joints add)
       (with-recursive-lock (*robot-mutex*)
         (add-links! *robot-model* links joints))))
    ((eql action (symbol-code 'iai_urdf_msgs-srv:alterurdf-request :remove))
     (ros-info (urdf-management) "Removing from robot model")
     (with-recursive-lock (*robot-mutex*)
       (remove-links! *robot-model* (coerce remove 'list))))))

(defun alter-urdf-service ()
  "Registers the service to alter the robot description."
  (setf *robot-model* (parse-urdf (get-param "robot_description" *default-description*)))
  (setf *urdf-pub* (advertise "/dynamic_robot_description" 'std_msgs-msg:String :latch t))
  (publish-urdf)
  (register-service *main-service-name* 'AlterUrdf)
  (ros-info (urdf-management) "Ready to alter urdf."))

(def-service-callback UrdfAlterUrdf (action urdf joint_description prefix)
  (cond
    ((eql action (symbol-code 'iai_urdf_msgs-srv:alterurdf-request :add))
     (ros-info (urdf-management) "Adding urdf to robot description.")
     (with-recursive-lock (*robot-mutex*)
       (attach-robot! *robot-model* (parse-urdf (get-absolute-path urdf))
                      (xml->joint joint_description) prefix))
     (make-response :success t))
    ((eql action (symbol-code 'iai_urdf_msgs-srv:alterurdf-request :remove))
     (make-response :success t))
    (t (ros-error (urdf-management simple-service) "Action ~a undefined." action)
       (make-response :success nil))))

(defun urdf-alter-urdf-service ()
  "Registers the service to alter the robot description."
  (register-service *urdf-service-name* 'UrdfAlterUrdf))

(defun publish-urdf ()
  "Generates an urdf description from the robot model and publishes it."
  (with-recursive-lock (*robot-mutex*)
    (publish-msg *urdf-pub* :data (generate-urdf-string *robot-model*))))

(defun call-alter-urdf (action add remove &optional (timeout 5))
  "Calls the service alter_urdf with `action', `add' and `remove' as parameters."
  (let ((found-service (wait-for-service *main-service-name* timeout)))
   (if found-service
       (let ((response (call-service *main-service-name* 'iai_urdf_msgs-srv:alterurdf 
                               :action action
                               :xml_elements_to_add add
                               :element_names_to_remove remove)))
         (unless (success response)
           (ros-warn (urdf-management simple-service) "AlterUrdf service didn't succeed."))
         (success response))
       (progn
         (ros-error (urdf-management simple-service) "No service ~a found." *main-service-name*)
         nil))))
