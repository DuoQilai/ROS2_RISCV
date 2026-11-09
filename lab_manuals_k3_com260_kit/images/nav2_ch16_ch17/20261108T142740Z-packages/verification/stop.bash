#!/usr/bin/env bash
set -euo pipefail
run=20261108T142740Z-packages
script=~/ROS2_RISCV_COM260_CH16_CH17/course_support/k3_com260_kit/scripts/x86-ch16-ch17.bash
printf 'RUN_ID=%s\n' "$run"
bash "$script" stop "$run"
bash "$script" stop "$run"
remaining=$(podman ps --filter label=org.ros2-riscv.component=chapter-gazebo --format '{{.Names}}')
[[ -z $remaining ]]
DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority xdpyinfo >/dev/null
printf 'PUBLIC_STOP_PASSED course_containers=0 desktop_ready=true\n'
