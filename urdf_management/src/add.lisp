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

(defun add-to-robot (xml robot parent-link-tree-original)
  "Adds the links and joints descripted to the robot model."
  (let ((parsed-xml (s-xml:parse-xml-string (format nil "<container>~a</container>" xml)
                                            :output-type :xml-struct))
        (parent-link-tree (alexandria:copy-hash-table parent-link-tree-original))
        (link-descriptions nil)
        (joint-descriptions nil)
        (root-name nil))

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
        (let ((joint-parent (s-xml:xml-element-attribute parent-desc :|link|))
              (joint-child (s-xml:xml-element-attribute child-desc :|link|)))
          (unless joint-parent
            (ros-warn (urdf-management) "No parent name for joint: ~a" joint-desc)
            (return-from add-to-robot nil))
          (unless joint-child
            (ros-warn (urdf-management) "No child name for joint: ~a" joint-desc)
            (return-from add-to-robot nil))
          (setf (gethash joint-child parent-link-tree) joint-parent))))

    ;; Check the parent-link-tree for circles and root links
    (multiple-value-bind (valid root err) (valid-tree parent-link-tree)
      (unless valid
        (ros-warn (urdf-management) "~a" err)
        (return-from add-to-robot nil))
      (setf root-name root))
 
    ;; Create the links from the description
    (dolist (link-desc link-descriptions)
      (let* ((link-name (s-xml:xml-element-attribute link-desc :|name|))
             (old-link (gethash link-name (links robot))))
        (unless (nth-value 1 (gethash link-name parent-link-tree))
          (progn
            (ros-error (urdf-management) "Link ~a not connnected to robot model." link-desc)
            (return-from add-to-robot nil)))
        (if old-link
            (setf (gethash link-name (links robot)) (update-link old-link link-desc robot))
            (let ((link (create-link link-desc robot)))
              (unless link
                (ros-warn (urdf-management) "Invalid link description: ~a" link-desc)
                (return-from add-to-robot nil))
              (setf (gethash link-name (links robot)) link)))))
    
    ;; Set root link
    (setf (slot-value robot 'root-link) (gethash root-name (links robot)))
 
    ;; Create the joints from the description and add them to the robot model
    (dolist (joint-desc joint-descriptions)
      (let* ((joint-name (s-xml:xml-element-attribute joint-desc :|name|))
             (old-joint (gethash joint-name (joints robot))))
        (when old-joint
          (setf (gethash joint-name (joints robot)) (update-joint old-joint joint-desc robot)))
        (unless old-joint
          (let ((joint (create-joint joint-desc robot)))
            (unless joint
              (ros-warn (urdf-management) "Invalid joint description: ~a" joint-desc)
              (return-from add-to-robot nil))
            (setf (gethash (name joint) (joints robot)) joint)))))

    (values t parent-link-tree)))

(defun create-link (link-desc robot)
  "Parses the xml description of the link. If it's a valid link the link is returned else nil."
  (let ((link (cl-urdf::parse-xml-node :|link| link-desc robot)))
    link))

(defun create-joint (joint-desc robot)
  "Parses the xml description of the joint. If it's a valid joint the joint is returned else nil."
  (let ((joint (cl-urdf::parse-xml-node :|joint| joint-desc robot)))
    joint))    
