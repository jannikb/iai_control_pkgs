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

(defun remove-from-robot (link-names robot)
  "Searches for links and joints with the given names in the robot model and removes them."
  (flet ((get-link (name) 
           (let ((link (gethash name (links robot))))
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
        (remove-link link robot))))
    t)

(defun remove-link (link robot)
  "Removes the `link' from the robot-model. Returns t if successfull."
  (let* ((link (gethash (name link) (links robot)))
         (parent-joint (from-joint link))
         (parent-link (parent parent-joint)))
    (unless (to-joints link)
      (setf (slot-value parent-link 'to-joints) 
            (remove-if (lambda (to-joint)
                         (equal (name to-joint) (name parent-joint)))
                       (to-joints parent-link)))
      (remhash (name parent-joint) (joints robot))    
      (remhash (name link) (links robot))
      t)))
