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

(defun start-urdf-service ()
  (with-ros-node ("urdf_management/urdf_service" :spin t)
    (urdf-alter-urdf-service)
    (ros-info (urdf_management) "Started ~a" *urdf-service-name*)))

(def-service-callback UrdfAlterUrdf (action urdf joint_description prefix)
  (let ((success nil))
    (cond
      ((eql action (symbol-code 'iai_urdf_msgs-srv:alterurdf-request :add))
       (ros-info (urdf-management) "Adding urdf to robot description.")
       (let ((description (urdf-to-attach urdf joint_description
                                          (unless (equal prefix "") prefix))))
         (setf success (call-alter-urdf action description ""))))
      ((eql action (symbol-code 'iai_urdf_msgs-srv:alterurdf-request :remove))
         (setf success (call-alter-urdf action "" (get-urdf-links urdf prefix))))
      (t (ros-error (urdf-management simple-service) "Action ~a undefined." action)
         (setf success nill)))
    (make-response :success success)))

(defun urdf-alter-urdf-service ()
  "Registers the service to alter the robot description."
  (register-service *urdf-service-name* 'UrdfAlterUrdf))
