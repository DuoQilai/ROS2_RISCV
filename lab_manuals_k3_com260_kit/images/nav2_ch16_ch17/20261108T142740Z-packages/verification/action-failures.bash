#!/usr/bin/env bash
set -eo pipefail
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
export ROS_DOMAIN_ID=184
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
python3 ~/ros2_course_com260_ch16_ch17_ws/verify_action_failures.py
