#!/usr/bin/env bash
set -eo pipefail
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
python3 ~/ros2_course_com260_ch16_ch17_ws/verify_navigation_final.py cancel
python3 ~/ros2_course_com260_ch16_ch17_ws/verify_navigation_final.py signal
