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
  (let* ((new-robot (add-link robot link joint))
         (new-link (gethash (name link) new-robot))
         (new-joint (gethash (name joint) new-robot)))
    (assert-equal (from-joint new-link) new-joint)
    (assert-equal (child new-joint) new-link)
    (assert-true (member joint (parent joint)))))

(defun test-remove-link (robot link-name)
  (let* ((old-link (gethash link-name (links robot)))
         (joint-name (name (from-joint old-link)))
         (parent-name (parent-name (from-joint old-link)))
         (new-robot (remove-link robot link-name)))
    (assert-false (gethash link-name (links new-robot)))
    (assert-false (gethash (name (from-joint old-link)) (joints new-robot)))
    (assert-false (member joint-name (to-joints (gethash parent-name (links new-robot)))
                          :key #'name :test #'equal))))


(defun test-replace-link (robot old-link-name new-link)
  (let ((old-link (gethash old-link-name (links robot)))
        (new-robot (replace-link robot old-link-name new-link)))
    (unless old-link
      (assert-false new-robot))
    (when old-link
      (assert-true (gethash (name new-link) (links new-robot)))
      (assert-equal (from-joint old-link) (from-joint new-link))
      (assert-true (every (lambda (joint) (member joint (to-joints new-link)))
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
    (test-replace-link robot "link2" link)))

(define-test pr2-replace-link
  (let ((robot (load-robot "pr2.urdf"))
        (link (make-instance 'link :name "r_elbow_flex_link")))
    (test-replace-link robot "r_elbow_flex_link" link)))
