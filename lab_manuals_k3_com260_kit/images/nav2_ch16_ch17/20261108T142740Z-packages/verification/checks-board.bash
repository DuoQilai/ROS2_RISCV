#!/usr/bin/env bash
set -eo pipefail
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
export ROS_DOMAIN_ID=182
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
cd ~/ros2_course_com260_ch16_ch17_ws/runs/20261108T142740Z-packages
printf 'RUN_ID=20261108T142740Z-packages\n'
ros2 run costmap_demo costmap_demo
ros2 run costmap_demo inflation_plot
python3 ~/ros2_course_com260_ch16_ch17_ws/verify_examples.py
