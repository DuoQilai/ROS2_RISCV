#!/usr/bin/env bash
set -euo pipefail
printf 'RUN_ID=20261108T142740Z-packages\n'
ssh -tt -o BatchMode=yes com260 'set -e; source ~/.config/ros2-course-com260-ch16-ch17/env.bash; python3 ~/ros2_course_com260_ch16_ch17_ws/inspect_costmap.py baseline'
ssh -tt -o BatchMode=yes duomaomao 'python3 ~/ros2_course_com260_ch16_ch17_humble_ws/box.py add'
sleep 8
ssh -tt -o BatchMode=yes com260 'source ~/.config/ros2-course-com260-ch16-ch17/env.bash; python3 ~/ros2_course_com260_ch16_ch17_ws/inspect_costmap.py added'
ssh -tt -o BatchMode=yes duomaomao 'DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority ffmpeg -hide_banner -loglevel error -f x11grab -window_id 0x2200013 -i :1.0 -frames:v 1 -y ~/ros2_course_com260_ch16_ch17_humble_ws/runs/20261108T142740Z-packages/costmap-added.png'
ssh -tt -o BatchMode=yes com260 'set -e; source ~/.config/ros2-course-com260-ch16-ch17/env.bash; ros2 param set /global_costmap/global_costmap inflation_layer.cost_scaling_factor 1.0; sleep 8; python3 ~/ros2_course_com260_ch16_ch17_ws/inspect_costmap.py scale-1; ros2 param set /global_costmap/global_costmap inflation_layer.cost_scaling_factor 10.0; sleep 8; python3 ~/ros2_course_com260_ch16_ch17_ws/inspect_costmap.py scale-10; ros2 param set /global_costmap/global_costmap inflation_layer.cost_scaling_factor 3.0'
ssh -tt -o BatchMode=yes duomaomao 'python3 ~/ros2_course_com260_ch16_ch17_humble_ws/box.py remove'
sleep 10
ssh -tt -o BatchMode=yes com260 'set -e; source ~/.config/ros2-course-com260-ch16-ch17/env.bash; python3 ~/ros2_course_com260_ch16_ch17_ws/inspect_costmap.py removed; ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"; ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"; python3 ~/ros2_course_com260_ch16_ch17_ws/verify_exercise_config.py'
ssh -tt -o BatchMode=yes duomaomao 'DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority ffmpeg -hide_banner -loglevel error -f x11grab -window_id 0x2200013 -i :1.0 -frames:v 1 -y ~/ros2_course_com260_ch16_ch17_humble_ws/runs/20261108T142740Z-packages/costmap-removed.png'
