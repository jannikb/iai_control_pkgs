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

(defparameter *default-description* "<robot name=\"default\"><link name=\"base_link\"/></robot>"
  "The robot description that is used if there is no description on the parameter server.")

(defvar *robot-model* nil)
(defvar *urdf-pub* nil)

(defun start-urdf-management ()
  (with-ros-node ("urdf_management" :spin t)
    (alter-urdf-service)))

(def-service-callback AlterUrdf (action xml_elements_to_add element_names_to_remove)
  (ros-info (urdf-management) "Altering robot description.")
  (let ((success (cond
                   ((eql action (symbol-code 'iai_urdf_msgs-srv:alterurdf-request :add))
                    (add-to-robot xml_elements_to_add *robot-model*))
                   ((eql action (symbol-code 'iai_urdf_msgs-srv:alterurdf-request :remove))
                    (remove-from-robot (coerce element_names_to_remove 'list) *robot-model*)))))
    (when success (publish-urdf))
    (make-response :success success)))

(defun alter-urdf-service ()
  "Registers the service to alter the robot description."
  (setf *robot-model* (parse-urdf (get-param "robot_description" *default-description*)))
  (setf *urdf-pub* (advertise "/dynamic_robot_description" 'std_msgs-msg:String :latch t))
  (publish-urdf)
  (register-service "alter_urdf" 'AlterUrdf)
  (ros-info (urdf-management) "Ready to alter urdf."))
  
(defun publish-urdf ()
  "Generates an urdf description from the robot model and publishes it."
  (publish-msg *urdf-pub* :data (generate-urdf-string *robot-model*)))
