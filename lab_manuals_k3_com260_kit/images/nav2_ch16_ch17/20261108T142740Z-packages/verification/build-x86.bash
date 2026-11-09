#!/usr/bin/env bash
set -euo pipefail
printf "RUN_ID=20261108T142740Z-packages\n"
export ROS_DOMAIN_ID=181
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.1.172"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.1.88"/></Peers></Discovery></Domain></CycloneDDS>'
cd ~/ROS2_RISCV_COM260_CH16_CH17
bash course_support/k3_com260_kit/scripts/x86-ch16-ch17.bash build
mkdir -p ~/ros2_course_com260_ch16_ch17_humble_ws/runs/20261108T142740Z-packages
cp ~/com260_ch16_ch17_validation/20261108T142740Z-packages/verify_build.py ~/ros2_course_com260_ch16_ch17_humble_ws/verify_build.py
podman run --rm --network none --volume "$HOME/ros2_course_com260_ch16_ch17_humble_ws:/workspace:rw" --workdir /workspace localhost/ros2-course-com260:ch06-nav2 bash -c 'source /workspace/install/setup.bash; python3 /workspace/verify_build.py /workspace' | tee ~/ros2_course_com260_ch16_ch17_humble_ws/runs/20261108T142740Z-packages/build-x86.json
