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

(defun update-link (link link-desc robot)
  (make-instance 'link
                 :name (name link)
                 :inertial (update-inertial (inertial link) 
                                            (cl-urdf::xml-element-child link-desc :|inertial|)
                                            robot)
                 :visual (update-visual (visual link)
                                        (cl-urdf::xml-element-child link-desc :|visual|)
                                        robot)
                 :collision (update-collision (collision link)
                                              (cl-urdf::xml-element-child link-desc :|collision|)
                                              robot)))

(defun update-inertial (inertial inertial-desc robot)
  (unless inertial-desc
    (return-from update-inertial inertial))
  (unless inertial
    (return-from update-inertial (cl-urdf::parse-xml-node :|inertial| inertial-desc robot)))
  (let ((mass-node (cl-urdf::xml-element-child inertial-desc :|mass|))
        (origin-node (cl-urdf::xml-element-child inertial-desc :|origin|)))
    (make-instance 'inertial
                   :origin (if origin-node
                               (cl-urdf::parse-xml-node :|origin| origin-node)
                               (origin inertial))
                   :mass (if mass-node
                             (cl-urdf::parse-xml-node :|mass| mass-node)
                             (mass inertial)))))

(defun update-visual (visual visual-desc robot)
  (unless visual-desc
    (return-from update-visual visual))
  (unless visual 
    (return-from update-visual (cl-urdf::parse-xml-node :|visual| visual-desc robot)))
  (let ((origin-node (cl-urdf::xml-element-child visual-desc :|origin|))
        (material-node (cl-urdf::xml-element-child visual-desc :|material|))
        (geometry-node (cl-urdf::xml-element-child visual-desc :|geometry|)))
    (make-instance 'visual 
                   :origin (if origin-node
                               (cl-urdf::parse-xml-node :|origin| origin-node)
                               (origin visual))
                   :material (if material-node 
                                 (cl-urdf::parse-xml-node :|material| material-node)
                                 (material visual))
                   :geometry (if geometry-node
                                 (cl-urdf::parse-xml-node :|geometry| geometry-node)
                                 (geometry visual)))))

(defun update-collision (collision collision-desc robot)
  (unless collision-desc
    (return-from update-collision collision))
  (unless collision
    (return-from update-collision (cl-urdf::parse-xml-node :|collision| collision-desc robot)))
  (let ((origin-node (cl-urdf::xml-element-child collision-desc :|origin|))
        (geometry-node (cl-urdf::xml-element-child collision-desc :|geometry|)))
    (make-instance 'collision
                   :origin (if origin-node
                               (cl-urdf::parse-xml-node :|origin| origin-node)
                               (origin collision))
                   :geometry (if geometry-node
                                 (cl-urdf::parse-xml-node :|geometry| geometry-node)
                                 (geometry collision)))))

(defun update-joint (joint joint-desc robot)
  (let ((type (s-xml:xml-element-attribute joint-desc :|type|))
        (axis-node (cl-urdf::xml-element-child joint-desc :|axis|))
        (origin-node (cl-urdf::xml-element-child joint-desc :|origin|))
        (limits-node (cl-urdf::xml-element-child joint-desc :|limit|))
        (parent-node (cl-urdf::xml-element-child joint-desc :|parent|))
        (child-node (cl-urdf::xml-element-child joint-desc :|child|)))
    (let ((parent-name (when parent-node (cl-urdf::parse-xml-node :|parent| parent-node)))
          (child-name (when child-node (cl-urdf::parse-xml-node :|child| child-node))))
      (make-instance 'joint
                     :name (name joint)
                     :type (if type
                               type
                               (joint-type joint))
                     :axis (if axis-node
                               (cl-urdf::parse-xml-node :|axis| axis-node)
                               (axis joint))
                     :origin (if origin-node
                                 (cl-urdf::parse-xml-node :|origin| origin-node)
                                 (origin joint))
                     :limits (if limits-node
                                (cl-urdf::parse-xml-node :|limit| limits-node)
                                (when (slot-boundp joint 'limits) (limits joint)))
                     :parent (if parent-name
                                 (gethash parent-name (links robot))
                                 (parent joint))
                     :child (if child-name
                                (gethash child-name (links robot))
                                (child joint))))))

