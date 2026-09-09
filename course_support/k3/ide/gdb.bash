#!/usr/bin/env bash
set -e
source "$HOME/.config/ros2-course-k3/env.bash"
source "$HOME/ros2_course_k3_ws/install/setup.bash"
exec /usr/bin/gdb "$@"
