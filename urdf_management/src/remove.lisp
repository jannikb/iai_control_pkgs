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

(defgeneric remove-link! (robot link)
  (:documentation ""))

(defmethod remove-link! ((robot robot) (link-name string))
  (let ((link (gethash link-name (links robot))))
    (when link
      (format t "Removing ~a~%" (name link))
      (flet ((remove-child (joint)
               (format t "Plan to remove ~a~%" (name (child joint)))
               (remove-link! robot (child joint))))
        (mapcar #'remove-child (to-joints link))
        (remhash (name (from-joint link)) (joints robot))
        (setf (to-joints (parent (from-joint link)))
              (remove (from-joint link) (to-joints (parent (from-joint link)))))
        (remhash (name link) (links robot)))
      robot)))

(defmethod remove-link! ((robot robot) (link link))
  (remove-link! robot (name link)))

(defun remove-link (robot link)
  (remove-link! (copy-object robot) link))

(defun remove-links! (robot links)
  (dolist (link links)
    (remove-link! robot link))
  t)

(defun remove-links (robot links)
  (remove-links! (copy-object robot) links))
