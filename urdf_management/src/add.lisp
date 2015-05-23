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

(defgeneric add-link (robot link joint)
  (:documentation "ad"))

;; (defmethod add-link ((robot logged-robot) (link link) (joint joint))
;;   (when (call-next-method)
;;     (let ((trans (make-instance 'robot-model-transition :action 'add-link
;;                                 :params (list link joint)))))))
    

(defmethod add-link ((robot robot) (link link) (joint joint))
  (unless (and (equal (child-name joint) (name link)))
               (gethash (name joint) (joints robot))
               (gethash (name link) (links robot))
    (return-from add-link nil))
  ;; Add link and joint
  (setf (gethash (name link) (links robot)) link)
  (setf (gethash (name joint) (joints robot)) joint)
  ;; Set other propoerties of the link and joint
  (setf (from-joint link) joint)
  (setf (child joint) link)
  (let ((parent-link (gethash (parent-name joint) (joints robot))))
    (setf (parent joint) parent-link)
    (setf (to-joints parent-link) (cons joint (to-joints parent-link))))
  robot)

                 
;; (defun add-to-robot (xml robot parent-link-tree-original)
;;   "Adds the links and joints descripted to the robot model."
;;   (let ((parsed-xml (parse-xml-string xml))
;;         (parent-link-tree (alexandria:copy-hash-table parent-link-tree-original))
;;         (link-descriptions nil)
;;         (new-link-names nil)
;;         (joint-descriptions nil)
;;         (root-name nil))

;;     ;; Get the descriptions of the joints and links from the xml
;;     (dolist (child (s-xml:xml-element-children parsed-xml))
;;       (when (not (typep child 's-xml::xml-element))
;;         (return-from add-to-robot nil))
;;       (case (s-xml:xml-element-name child)
;;         (:|link| (push child link-descriptions))
;;         (:|joint| (push child joint-descriptions))
;;         (otherwise         
;;          (ros-warn (urdf-management) 
;;                    "Ignoring element: ~a" child))))

;;     (setf new-link-names (mapcar (lambda (link-desc)
;;                                    (s-xml:xml-element-attribute link-desc :|name|))
;;                                  link-descriptions))

;;      ;; Check the joint descriptions and fill child-joints and joint-parents
;;     (dolist (joint-desc joint-descriptions)
;;       (let ((joint-name (s-xml:xml-element-attribute joint-desc :|name|)))
;;         (unless joint-name
;;           (ros-error (urdf-management) "No name for joint: '~a'" joint-desc)
;;           (return-from add-to-robot nil))
;;         (let ((old-joint (gethash joint-name (joints robot)))
;;               (joint-parent (get-joint-link joint-desc :|parent|))
;;               (joint-child (get-joint-link joint-desc :|child|)))
;;           (when (and old-joint (not joint-parent))
;;             (setf joint-parent (name (parent old-joint))))
;;           (unless joint-parent
;;             (ros-error (urdf-management) "No parent name for joint '~a'" joint-name)
;;             (return-from add-to-robot nil))
;;           (unless (or (gethash joint-parent (links robot))
;;                       (member joint-parent new-link-names :test 'equal))
;;             (ros-error (urdf-management) "Couldn't find parent '~a' of joint '~a'" 
;;                       joint-parent joint-name)
;;             (return-from add-to-robot nil))
;;           (when old-joint
;;             (if joint-child
;;                 (remhash (name (child old-joint)) parent-link-tree)
;;                 (setf joint-child (name (child old-joint)))))
;;           (unless joint-child
;;             (ros-error (urdf-management) "No child name for joint '~a'" joint-name)
;;             (return-from add-to-robot nil))
;;           (unless (or (gethash joint-child (links robot))
;;                       (member joint-child new-link-names :test 'equal))
;;             (ros-error (urdf-management) "Couldn't find child '~a' of joint '~a'" 
;;                       joint-child joint-name)
;;             (return-from add-to-robot nil))
;;           (setf (gethash joint-child parent-link-tree) joint-parent))))

;;     ;; Check the parent-link-tree for circles and root links
;;     (multiple-value-bind (valid root err) (valid-tree parent-link-tree)
;;       (unless valid
;;         (ros-warn (urdf-management) "~a" err)
;;         (return-from add-to-robot nil))
;;       (setf root-name root))

;;     ;; Check if all old links are still connected to the robot
;;     (alexandria:maphash-keys (lambda (child)
;;                                 (unless (nth-value 1 (gethash child parent-link-tree))
;;                                   (ros-error (urdf-management) 
;;                                              "Link ~a not connnected to robot model anymore." 
;;                                              child)
;;                                   (return-from add-to-robot nil)))
;;                              (links robot))
 
;;     ;; Create the links from the description
;;     (dolist (link-desc link-descriptions)
;;       (let* ((link-name (s-xml:xml-element-attribute link-desc :|name|))
;;              (old-link (gethash link-name (links robot))))
;;         (unless (nth-value 1 (gethash link-name parent-link-tree))
;;           (ros-error (urdf-management) "Link ~a not connnected to robot model." link-desc)
;;           (return-from add-to-robot nil))
;;         (if old-link
;;             (setf (gethash link-name (links robot)) (update-link old-link link-desc robot))
;;             (let ((link (create-link link-desc robot)))
;;               (unless link
;;                 (ros-error (urdf-management) "Invalid link description: ~a" link-desc)
;;                 (return-from add-to-robot nil))
;;               (setf (gethash link-name (links robot)) link)))))
    
;;     ;; Set root link
;;     (setf (slot-value robot 'root-link) (gethash root-name (links robot)))
 
;;     ;; Create the joints from the description and add them to the robot model
;;     (dolist (joint-desc joint-descriptions)
;;       (let* ((joint-name (s-xml:xml-element-attribute joint-desc :|name|))
;;              (old-joint (gethash joint-name (joints robot))))
;;          (when old-joint
;;           (setf (gethash joint-name (joints robot)) (update-joint old-joint joint-desc robot)))
;;         (unless old-joint
;;           (let ((joint (create-joint joint-desc robot)))
;;             (unless joint
;;               (ros-warn (urdf-management) "Invalid joint description: ~a" joint-desc)
;;               (return-from add-to-robot nil))
;;             (setf (gethash (name joint) (joints robot)) joint)))))

;;     (values t parent-link-tree)))

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

;; (defun create-joint (joint-desc robot)
;;   "Parses the xml description of the joint. If it's a valid joint the joint is returned else nil."
;;   (let ((joint (cl-urdf::parse-xml-node :|joint| joint-desc robot)))
;;     joint))  

;; (defun get-joint-link (joint-desc element)
;;   (let ((link-desc (cl-urdf::xml-element-child joint-desc element)))
;;     (when link-desc
;;       (s-xml:xml-element-attribute link-desc :|link|))))
