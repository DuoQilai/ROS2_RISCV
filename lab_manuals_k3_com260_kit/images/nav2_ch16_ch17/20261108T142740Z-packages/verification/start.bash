#!/usr/bin/env bash
set -euo pipefail
export ROS_DOMAIN_ID=181
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.1.172"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.1.88"/></Peers></Discovery></Domain></CycloneDDS>'
cd ~/ROS2_RISCV_COM260_CH16_CH17
script=course_support/k3_com260_kit/scripts/x86-ch16-ch17.bash
bash "$script" start 20261108T142740Z-packages
for attempt in {1..20}; do
  state=$(timeout 8 bash "$script" exec 20261108T142740Z-packages ros2 lifecycle get /bt_navigator 2>&1) || true
  printf '%s
' "$state"
  [[ "$state" == *'active [3]'* ]] && break
  sleep 2
done
[[ "$state" == *'active [3]'* ]]
bash "$script" exec 20261108T142740Z-packages ros2 lifecycle get /global_costmap/global_costmap
bash "$script" exec 20261108T142740Z-packages ros2 param get /global_costmap/global_costmap resolution
bash "$script" exec 20261108T142740Z-packages ros2 param get /planner_server GridBased.tolerance
bash "$script" exec 20261108T142740Z-packages ros2 param get /controller_server general_goal_checker.xy_goal_tolerance
bash "$script" exec 20261108T142740Z-packages ros2 param get /controller_server FollowPath.xy_goal_tolerance
bash "$script" exec 20261108T142740Z-packages ros2 param get /controller_server progress_checker.required_movement_radius
printf 'PUBLIC_START_EXEC_PASSED
'
