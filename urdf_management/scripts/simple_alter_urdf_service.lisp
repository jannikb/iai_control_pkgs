#!/usr/bin/env sh
"true"; exec /usr/bin/env sbcl --noinform --load `rospack find roslisp`/scripts/roslisp-sbcl-init --script "$0" "$@"

(ros-load:load-system "urdf_management" "urdf-management")

(urdf-management:start-simple-service)
