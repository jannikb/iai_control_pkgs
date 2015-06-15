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

(defgeneric replace-link! (robot old-link link)
  (:documentation ""))

(defgeneric replace-joint! (robot old-joint joint)
  (:documentation ""))

(defmethod replace-link! ((robot robot) (old-link-name string) (link link))
  (let ((old-link (gethash old-link-name (links robot))))
    (when old-link
      (replace-link! robot old-link link))))
  
(defmethod replace-link! ((robot robot) (old-link link) (link link))
  (setf (child (from-joint old-link)) link)
  (mapcar (lambda (to-joint) (setf (parent to-joint) link))
          (to-joints old-link))
  (setf (gethash (name link) (links robot)) link)
  (setf (from-joint link) (from-joint old-link))
  (setf (to-joints link) (to-joints old-link))
  robot)


(defmethod replace-joint! ((robot robot) (old-joint-name string) (joint joint))
  (let ((old-joint (gethash old-joint-name (joints robot))))
    (when old-joint
      (replace-joint! robot old-joint joint))))
  
(defmethod replace-joint! ((robot robot) (old-joint joint) (joint joint))
  (when (and (equal (parent-name joint) (parent-name old-joint))
             (equal (child-name joint) (child-name old-joint)))
    (setf (parent joint) (parent old-joint))
    (setf (child joint) (child old-joint))
    (setf (gethash (name joint) (joints robot)) joint)
    (setf (to-joints (parent joint))
          (cons joint (remove old-joint (to-joints (parent joint)))))
    robot))

(defun replace-link (robot old-link link)
  (replace-link! (copy-object robot) old-link (copy-object link)))

(defun replace-joint (robot old-joint joint)
  (replace-joint! (copy-object robot) old-joint (copy-object joint)))
