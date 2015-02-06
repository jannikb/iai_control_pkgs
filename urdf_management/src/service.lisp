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
                    (add-to-robot xml_elements_to_add))
                   ((eql action (symbol-code 'iai_urdf_msgs-srv:alterurdf-request :remove))
                    (remove-from-robot element_names_to_remove)))))
    (when success (publish-urdf))
    (make-response :success success)))

(defun alter-urdf-service ()
  "Registers the service to alter the robot description."
  (setf *robot-model* (parse-urdf (get-param "robot_description" *default-description*)))
  (setf *urdf-pub* (advertise "/dynamic_robot_description" 'std_msgs-msg:String :latch t))
  (publish-urdf)
  (register-service "alter_urdf" 'AlterUrdf)
  (ros-info (urdf-management) "Ready to alter urdf."))

(defun add-to-robot (xml)
  "Adds the links and joints descripted to the robot model."
  (let ((parsed-xml (s-xml:parse-xml-string (format nil "<container>~a</container>" xml)
                                            :output-type :xml-struct))
        (link-descriptions nil)
        (joint-descriptions nil)
        (new-links nil)
        (joint-parents nil)
        (child-joints nil))

    ;; Get the descriptions of the joints and links from the xml
    (dolist (child (s-xml:xml-element-children parsed-xml))
      (when (not (typep child 's-xml::xml-element))
        (return-from add-to-robot nil))
      (case (s-xml:xml-element-name child)
        (:|link| (push child link-descriptions))
        (:|joint| (push child joint-descriptions))
        (otherwise         
         (ros-error (urdf-management) 
                    "Description contains an illegal element: ~a" child)
         (return-from add-to-robot nil))))
 
    ;; Check the joint dscriptions and fill child-joints and joint-parents
    (dolist (joint-desc joint-descriptions)
      (let ((joint-name (s-xml:xml-element-attribute joint-desc :|name|))
            (parent-desc (cl-urdf::xml-element-child joint-desc :|parent|))
            (child-desc (cl-urdf::xml-element-child joint-desc :|child|)))
        (unless parent-desc
          (ros-error (urdf-management) "No parent for joint: ~a" joint-desc)
          (return-from add-to-robot nil))
        (unless joint-name
          (ros-error (urdf-management) "No name for joint: ~a" joint-desc)
          (return-from add-to-robot nil))
        (let ((joint-parent  (s-xml:xml-element-attribute parent-desc :|link|))
              (joint-child  (s-xml:xml-element-attribute child-desc :|link|)))
          (unless joint-parent
            (ros-error (urdf-management) "No parent name for joint: ~a" joint-desc)
            (return-from add-to-robot nil))
          (unless joint-child
            (ros-error (urdf-management) "No child name for joint: ~a" joint-desc)
            (return-from add-to-robot nil))
          (push joint-parent joint-parents)
          (push (intern joint-name) joint-parents)
          (push joint-name child-joints)
          (push (intern joint-child) child-joints))))       
 
    ;; Create the links from the description
    (dolist (link-desc link-descriptions)
      (let ((link (create-link link-desc)))
        (unless link
          (ros-error (urdf-management) "Invalid link description: ~a" link-desc)
          (return-from add-to-robot nil))
        (push link new-links)))

    ;; Add the new links to the robot-model
    (dolist (link new-links)
      (if (connected-to-robot (name link) child-joints joint-parents (length new-links))
          (add-link-to-robot link)
          (progn
            (ros-error (urdf-management) "No connection between root link and ~a" link)
            (return-from add-to-robot nil))))
 
    ;; Create the joints from the description and add them to the robot model
    (dolist (joint-desc joint-descriptions)
      (let ((joint (create-joint joint-desc)))
        (unless joint
          (ros-error (urdf-management) "Invalid joint description: ~a" joint-desc)
          (return-from add-to-robot nil))
        (add-joint-to-robot joint)))

    t))

(defun create-link (link-desc)
  "Parses the xml description of the link. If it's a valid link the link is returned else nil."
  (let ((link (cl-urdf::parse-xml-node :|link| link-desc *robot-model*)))
    link))

(defun add-link-to-robot (link)
  "Adds the `link' to the robot model."
  (let ((link-table (links *robot-model*)))
    (setf (gethash (name link) link-table) link)))

(defun create-joint (joint-desc)
  "Parses the xml description of the joint. If it's a valid joint the joint is returned else nil."
  (let ((joint (cl-urdf::parse-xml-node :|joint| joint-desc *robot-model*)))
    joint))

(defun add-joint-to-robot (joint)
  "Adds the `joint' to the robot model."
  (let ((joint-table (joints *robot-model*)))
    (setf (gethash (name joint) joint-table) joint)))

(defun remove-from-robot (names)
  "Searches for links and joints with the given names in the robot model and removes them."
  (let ((link-table (links *robot-model*))
        (joint-table (joints *robot-model*)))
    (map 'vector (lambda (name)
                   (remhash name link-table)
                   (remhash name joint-table))
         names)
    t))

(defun connected-to-robot (link-name child-joints joint-parents num-of-links &optional (depth 0))
  (when (< depth num-of-links)
    (let ((from-joint (getf child-joints (intern link-name))))
      (when from-joint
        (let ((from-link (getf joint-parents (intern from-joint))))
          (when from-link
            (if (gethash from-link (links *robot-model*))
                t
                (connected-to-robot from-link child-joints joint-parents 
                                    num-of-links (1+ depth)))))))))
  
(defun publish-urdf ()
  "Generates an urdf description from the robot model and publishes it."
  (publish-msg *urdf-pub* :data (generate-urdf-string *robot-model*)))
