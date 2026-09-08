#!/usr/bin/env bash
set -e
source "$HOME/.config/ros2-course/env.bash"
source "$HOME/ros2_course_ws/install/setup.bash"
exec /usr/bin/gdb "$@"
