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

(defgeneric remove-link (robot link)
  (:documentation ""))

(defmethod remove-link ((robot robot) (link-name string))
  (let ((link (gethash link-name (links robot))))
    (when link
      (flet ((remove-child (joint)
               (remove-link robot (child joint))))
        (mapcar #'remove-child (to-joints link))
        (remhash (name (from-joint link)) (joints robot))
        (setf (to-joints (parent (from-joint link)))
              (remove (from-joint link) (to-joints (parent (from-joint link)))))
        (remhash (name link) (links robot)))
      robot)))

(defmethod remove-link ((robot robot) (link link))
  (remove-link robot (name link)))

;; (defun remove-from-robot (link-names robot parent-link-tree-original)
;;   "Searches for links and joints with the given names in the robot model and removes them."
;;   (let ((parent-link-tree (alexandria:copy-hash-table parent-link-tree-original)))
;;     (dolist (link-name link-names)
;;       (remhash link-name parent-link-tree))
;;     (dolist (link-name (alexandria:hash-table-keys parent-link-tree))
;;       (when (member (gethash link-name parent-link-tree) link-names :test 'equal)
;;         (setf (gethash link-name parent-link-tree) nil)))

;;     ;; Check the parent-link-tree for circles and root links
;;     (multiple-value-bind (valid root err) (valid-tree parent-link-tree)
;;       (unless valid
;;         (ros-warn (urdf-management) "~a" err)
;;         (return-from remove-from-robot nil))
;;       (setf (slot-value robot 'root-link) (gethash root (links robot))))

;;     ;; Remove the links from the robot model
;;     (dolist (link-name link-names)
;;       (if (gethash link-name (links robot))
;;           (remove-link link-name robot)
;;           (ros-warn (urdf-management) "Link ~a not found" link-name)))

;;     (values t parent-link-tree)))

;; (defun remove-link (link-name robot)
;;   "Removes the `link' from the robot-model. Returns t if successfull."
;;   (let* ((link (gethash link-name (links robot)))
;;          (parent-joint (from-joint link))
;;          (child-joints (to-joints link)))
;;     (when parent-joint
;;       (let ((parent-link (parent parent-joint)))
;;         ;; remove the parent joint from the parent link's to-joints
;;         (setf (slot-value parent-link 'to-joints) 
;;               (remove-if (lambda (to-joint)
;;                            (equal (name to-joint) (name parent-joint)))
;;                          (to-joints parent-link)))
;;         (remhash (name parent-joint) (joints robot))))
;;     (dolist (child-joint child-joints)
;;       (setf (slot-value (child child-joint) 'from-joint) nil)
;;       (remhash (name child-joint) (joints robot)))            
;;     (remhash (name link) (links robot))
;;     t))
