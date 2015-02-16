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
                    (remove-from-robot (coerce element_names_to_remove 'list))))))
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
 
    ;; Check the joint descriptions and fill child-joints and joint-parents
    (dolist (joint-desc joint-descriptions)
      (let ((joint-name (s-xml:xml-element-attribute joint-desc :|name|))
            (parent-desc (cl-urdf::xml-element-child joint-desc :|parent|))
            (child-desc (cl-urdf::xml-element-child joint-desc :|child|)))
        (unless parent-desc
          (ros-warn (urdf-management) "No parent for joint: ~a" joint-desc)
          (return-from add-to-robot nil))
        (unless joint-name
          (ros-warn (urdf-management) "No name for joint: ~a" joint-desc)
          (return-from add-to-robot nil))
        (let ((joint-parent  (s-xml:xml-element-attribute parent-desc :|link|))
              (joint-child  (s-xml:xml-element-attribute child-desc :|link|)))
          (unless joint-parent
            (ros-warn (urdf-management) "No parent name for joint: ~a" joint-desc)
            (return-from add-to-robot nil))
          (unless joint-child
            (ros-warn (urdf-management) "No child name for joint: ~a" joint-desc)
            (return-from add-to-robot nil))
          (push joint-parent joint-parents)
          (push (intern joint-name) joint-parents)
          (push joint-name child-joints)
          (push (intern joint-child) child-joints))))       
 
    ;; Create the links from the description
    (dolist (link-desc link-descriptions)
      (let* ((link-name (s-xml:xml-element-attribute link-desc :|name|))
             (old-link (gethash link-name (links *robot-model*))))
        (when old-link
          (push (update-link old-link link-desc) new-links))
        (unless old-link
          (let ((link (create-link link-desc)))
            (unless link
              (ros-warn (urdf-management) "Invalid link description: ~a" link-desc)
              (return-from add-to-robot nil))
            (push link new-links)))))

    ;; Add the new links to the robot-model
    (dolist (link new-links)
      (unless (connected-to-robot (name link) child-joints joint-parents (length new-links))
        (ros-warn (urdf-management) "No connection between root link and ~a" (name link))
        (return-from add-to-robot nil))
      (setf (gethash (name link) (links *robot-model*)) link))
 
    ;; Create the joints from the description and add them to the robot model
    (dolist (joint-desc joint-descriptions)
      (let* ((joint-name (s-xml:xml-element-attribute joint-desc :|name|))
             (old-joint (gethash joint-name (joints *robot-model*))))
        (when old-joint
          (setf (gethash joint-name (joints *robot-model*)) (update-joint old-joint joint-desc)))
        (unless old-joint
          (let ((joint (create-joint joint-desc)))
            (unless joint
              (ros-warn (urdf-management) "Invalid joint description: ~a" joint-desc)
              (return-from add-to-robot nil))
            (setf (gethash (name joint) (joints *robot-model*)) joint)))))

    t))

(defun create-link (link-desc)
  "Parses the xml description of the link. If it's a valid link the link is returned else nil."
  (let ((link (cl-urdf::parse-xml-node :|link| link-desc *robot-model*)))
    link))

(defun update-link (link link-desc)
  (make-instance 'link
                 :name (name link)
                 :inertial (update-inertial (inertial link) 
                                            (cl-urdf::xml-element-child link-desc :|inertial|))
                 :visual (update-visual (visual link)
                                        (cl-urdf::xml-element-child link-desc :|visual|))
                 :collision (update-collision (collision link)
                                             (cl-urdf::xml-element-child link-desc :|collision|))))

(defun update-inertial (inertial inertial-desc)
  (unless inertial-desc
    (return-from update-inertial inertial))
  (unless inertial
    (return-from update-inertial (cl-urdf::parse-xml-node :|inertial| inertial-desc *robot-model*)))
  (let ((mass-node (cl-urdf::xml-element-child inertial-desc :|mass|))
        (origin-node (cl-urdf::xml-element-child inertial-desc :|origin|)))
    (make-instance 'inertial
                   :origin (if origin-node
                               (cl-urdf::parse-xml-node :|origin| origin-node)
                               (origin inertial))
                   :mass (if mass-node
                             (cl-urdf::parse-xml-node :|mass| mass-node)
                             (mass inertial)))))

(defun update-visual (visual visual-desc)
  (unless visual-desc
    (return-from update-visual visual))
  (unless visual 
    (return-from update-visual (cl-urdf::parse-xml-node :|visual| visual-desc *robot-model*)))
  (let ((origin-node (cl-urdf::xml-element-child visual-desc :|origin|))
        (material-node (cl-urdf::xml-element-child visual-desc :|material|))
        (geometry-node (cl-urdf::xml-element-child visual-desc :|geometry|)))
    (make-instance 'visual 
                   :origin (if origin-node
                               (cl-urdf::parse-xml-node :|origin| origin-node)
                               (origin visual))
                   :material (if material-node 
                                 (cl-urdf::parse-xml-node :|material| material-node)
                                 (material visual))
                   :geometry (if geometry-node
                                 (cl-urdf::parse-xml-node :|geometry| geometry-node)
                                 (geometry visual)))))

(defun update-collision (collision collision-desc)
  (unless collision-desc
    (return-from update-collision collision))
  (unless collision
    (return-from update-collision (cl-urdf::parse-xml-node :|collision| collision-desc *robot-model*)))
  (let ((origin-node (cl-urdf::xml-element-child collision-desc :|origin|))
        (geometry-node (cl-urdf::xml-element-child collision-desc :|geometry|)))
    (make-instance 'collision
                   :origin (if origin-node
                               (cl-urdf::parse-xml-node :|origin| origin-node)
                               (origin collision))
                   :geometry (if geometry-node
                                 (cl-urdf::parse-xml-node :|geometry| geometry-node)
                                 (geometry collision)))))

(defun update-joint (joint joint-desc)
  (let ((type (s-xml:xml-element-attribute joint-desc :|type|))
        (axis-node (cl-urdf::xml-element-child joint-desc :|axis|))
        (origin-node (cl-urdf::xml-element-child joint-desc :|origin|))
        (limits-node (cl-urdf::xml-element-child joint-desc :|limit|))
        (parent-node (cl-urdf::xml-element-child joint-desc :|parent|))
        (child-node (cl-urdf::xml-element-child joint-desc :|child|)))
    (let ((parent-name (when parent-node (cl-urdf::parse-xml-node :|parent| parent-node)))
          (child-name (when child-node (cl-urdf::parse-xml-node :|child| child-node))))
      (make-instance 'joint
                     :name (name joint)
                     :type (if type
                               type
                               (joint-type joint))
                     :axis (if axis-node
                               (cl-urdf::parse-xml-node axis-node :|axis|)
                               (axis joint))
                     :origin (if origin-node
                                 (cl-urdf::parse-xml-node origin-node :|origin|)
                                 (origin joint))
                     :limits (if limits-node
                                (cl-urdf::parse-xml-node limits-node :|limit|)
                                (when (slot-boundp joint 'limits) (limits joint)))
                     :parent (if parent-name
                                 (gethash parent-name (links *robot-model*))
                                 (parent joint))
                     :child (if child-name
                                (gethash child-name (links *robot-model*))
                                (child joint))))))

(defun create-joint (joint-desc)
  "Parses the xml description of the joint. If it's a valid joint the joint is returned else nil."
  (let ((joint (cl-urdf::parse-xml-node :|joint| joint-desc *robot-model*)))
    joint))    

(defun remove-from-robot (link-names)
  "Searches for links and joints with the given names in the robot model and removes them."
  (flet ((get-link (name) 
           (let ((link (gethash name (links *robot-model*))))
             (unless link
               (ros-warn (urdf-management) "Link ~a not found." name)
               (return-from remove-from-robot nil))
             link))
         (find-link (link links) (find (name link) links :key #'name)))
    (let ((unremovable-links (mapcar #'get-link link-names))
          (still-unremovable-links nil)
          (removable-links nil))
      (loop while unremovable-links
            do (dolist (link unremovable-links)
                 (let ((to-joints (to-joints link)))
                   (if (or (not to-joints)
                           (reduce (lambda (a b) (and a b))
                                   (mapcar (lambda (joint)
                                             (find-link (child joint) removable-links))
                                           to-joints)))
                       (push link removable-links)
                       (push link still-unremovable-links))))
               (if (and still-unremovable-links
                        (equal unremovable-links (reverse still-unremovable-links)))
                   (progn
                     (ros-warn (urdf-management) "Cannot remove links ~a" 
                                                (mapcar #'name still-unremovable-links))
                     (return-from remove-from-robot nil))
                   (progn 
                     (setf unremovable-links still-unremovable-links)
                     (setf still-unremovable-links nil))))
      (dolist (link (reverse removable-links))
        (remove-link link))))
    t)

(defun remove-link (link)
  "Removes the `link' from the robot-model. Returns t if successfull."
  (let* ((link (gethash (name link) (links *robot-model*)))
         (parent-joint (from-joint link))
         (parent-link (parent parent-joint)))
    (unless (to-joints link)
      (setf (slot-value parent-link 'to-joints) 
            (remove-if (lambda (to-joint)
                         (equal (name to-joint) (name parent-joint)))
                       (to-joints parent-link)))
      (remhash (name parent-joint) (joints *robot-model*))    
      (remhash (name link) (links *robot-model*))
      t)))

(defun connected-to-robot (link-name child-joints joint-parents num-of-links &optional (depth 0))
  "Checks if a link is connected to a link of the robot-model."
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
