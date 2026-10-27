#!/usr/bin/env bash
set -e
source "$HOME/.config/ros2-course-com260/env.bash"
exec /usr/bin/gdb "$@"
