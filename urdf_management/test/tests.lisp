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

(in-package :urdf-management-test)

(defun load-robot (name)
  "Parses the urdf file with name `name' in the test_urdfs directory."
  (cl-urdf:parse-urdf (pathname (format nil "test_urdfs/~a" name))))

(defun test-add-link (robot link joint)
  (let ((parent-link (gethash (parent-name joint) (links robot))))
    (add-link robot link joint)
    (assert-equal (from-joint link) joint)
    (assert-equal (child joint) link)
    (assert-true (member joint (to-joints parent-link)))))

(defun test-remove-link (robot link-name)
  (let* ((link (gethash link-name (links robot))))
      (assert-true link)
      (when link
        (remove-link robot link-name)
        (assert-false (gethash link-name (links robot)))
        (assert-false (gethash (name (from-joint link)) (joints robot)))
        (assert-false (member (from-joint link) (to-joints (parent (from-joint link))))))))
        ;(test-removed-to-joints link)))))

(defun test-replace-link (robot link)
  (let ((old-link (gethash (name link) (links robot)))
        (return-value (replace-link robot link)))
    (unless old-link
      (assert-false return-value))
    (when old-link
      (assert-equal (nth-value 0 (gethash (name link) (links robot)))
                    link)
      (assert-equal (from-joint old-link) (from-joint link))
      (assert-true (every (lambda (joint) (member joint (to-joints link)))
                          (to-joints old-link))))))
  

(define-test simple-robot-add-link
  (let ((robot (load-robot "simple_robot.urdf"))
        (link (make-instance 'link :name "link3"))
        (joint (make-instance 'joint :name "joint2"
                              :parent-name "link2"
                              :child-name "link3")))
    (test-add-link robot link joint)))

(define-test simple-robot-remove-link
  (let ((robot (load-robot "simple_robot.urdf")))
    (test-remove-link robot "link2")))

(define-test pr2-remove-link
  (let ((robot (load-robot "pr2.urdf")))
    (test-remove-link robot "r_elbow_flex_link")))

(define-test simple-robot-replace-link
  (let ((robot (load-robot "simple_robot.urdf"))
        (link (make-instance 'link :name "link2")))
    (test-replace-link robot link)))

(define-test pr2-replace-link
  (let ((robot (load-robot "pr2.urdf"))
        (link (make-instance 'link :name "r_elbow_flex_link")))
    (test-replace-link robot link)))
