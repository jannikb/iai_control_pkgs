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

(defun get-tree (robot)
  (let ((parent-link-tree (make-hash-table :test 'equal))
        (root (root-link robot)))
    (setf (gethash (name root) parent-link-tree) nil)
    (dolist (joint (alexandria:hash-table-values (joints robot)))
      (let ((parent (parent joint))
            (child (child joint)))
        (setf (gethash (name child) parent-link-tree) (when parent (name parent)))))
    parent-link-tree))

(defun valid-tree (tree)
  (let ((root nil)
        (in-tree (make-hash-table :test 'equal :size (hash-table-count tree)))
        (links (alexandria:hash-table-keys tree)))
    (dolist (link links)
      (let ((new-root (add-to-tree link tree in-tree root)))
        (if new-root
          (if root
              (unless (equal new-root root)
                (return-from valid-tree 
                  (values nil "" (format nil "Found two root links ~a and ~a" root new-root))))
              (setf root new-root))
          (return-from valid-tree (values nil "" 
                                          (format nil "Error parsing tree for link ~a" link))))))
    (values t root "")))

(defun add-to-tree (link tree in-tree old-root &optional (depth 0))
  (unless (> depth (hash-table-count tree))
    (if (gethash link in-tree)
        old-root
        (multiple-value-bind (parent in-tree-p) (gethash link tree)
          (when in-tree-p
            (if parent
                (let ((root (add-to-tree parent tree in-tree old-root (1+ depth))))
                  (setf (gethash link in-tree) t)
                  root)
                link))))))
  
