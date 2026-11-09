#!/usr/bin/env bash
set -eo pipefail
printf "RUN_ID=20261108T142740Z-packages\n"
export ROS_DOMAIN_ID=181
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.1.88"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.1.172"/></Peers></Discovery></Domain></CycloneDDS>'
cd ~/ROS2_RISCV_COM260_CH16_CH17
bash setup_course_k3_com260_kit.sh --build-ch16
bash setup_course_k3_com260_kit.sh --build-ch17
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
mkdir -p ~/ros2_course_com260_ch16_ch17_ws/runs/20261108T142740Z-packages
python3 ~/com260_ch16_ch17_validation/20261108T142740Z-packages/verify_build.py ~/ros2_course_com260_ch16_ch17_ws | tee ~/ros2_course_com260_ch16_ch17_ws/runs/20261108T142740Z-packages/build-board.json
