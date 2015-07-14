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

(defun add-link! (robot link joint)
  "Uses the `joint' to connect the `link' to the `robot'. Modifies and returns `robot'."
  (when (and robot link joint
             (equal (child-name joint) (name link))
             (not (gethash (name joint) (joints robot)))
             (not (gethash (name link) (links robot)))
             (gethash (parent-name joint) (links robot)))
    ;; Add link and joint
    (setf (gethash (name link) (links robot)) link)
    (setf (gethash (name joint) (joints robot)) joint)
    ;; Set other properties of the link and joint
    (setf (from-joint link) joint)
    (setf (child joint) link)
    (let ((parent-link (gethash (parent-name joint) (links robot))))
      (setf (parent joint) parent-link)
      (setf (to-joints parent-link) (cons joint (to-joints parent-link))))
    robot))

(defun add-link (robot link joint)
  "Uses the `joint' to connect the `link' to the `robot'. Doesn't modify `robot' and instead returns a new object of type robot."
  (add-link! (copy-object robot) (copy-object link) (copy-object joint)))

(defun add-links! (robot links joints)
  "Uses the `joints' to connect the `links' to the `robot'. Modifies and returns `robot'. Returns nil if it fails but might have added some of the `links'."
  (let ((joint-backlog nil)
        (prev-backlog-length (1+ (length joints))))
    (loop while (and joints
                     (< (length joints) prev-backlog-length))
          do
             (setf prev-backlog-length (length joints))
             (loop while joints do
               (unless (and (parent-name (car joints)) (child-name (car joints)))
                 (ros-error (urdf-management) "Malformed joint: No parent or child name.")
                 (return-from add-links! nil))
               (let* ((joint (pop joints))
                      (parent (gethash (parent-name joint) (links robot))))
                 (when parent
                   (let ((link (find (child-name joint) links :key #'name :test #'equal)))
                     (when link
                       (add-link! robot link joint))
                     (unless link
                       (ros-warn (urdf_management) "Link ~a not found." (child-name joint)))))
                 (unless parent
                   (push joint joint-backlog))))
             (setf joints joint-backlog)
             (setf joint-backlog nil))
    robot))

(defun add-links (robot links joints)
  "Uses the `joints' to connect the `links' to the `robot'. Doesn't modifiy `robot' instead a new object of type robot is returned or nil if it failed."
  (add-links! (copy-object robot)
              (copy-object links)
              (copy-object joints)))
