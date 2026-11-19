#!/usr/bin/env bash
set -Eeo pipefail
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID=204 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1
cd /data
ros2 bag record --use-sim-time -o euroc_recorded /camera/image_raw /imu > record.log 2>&1 &
recorder=$!
trap 'kill -INT "$recorder" 2>/dev/null || true; wait "$recorder" 2>/dev/null || true' EXIT
sleep 2
ros2 bag play euroc_source --clock 50 --delay 3 > replay.log 2>&1
sleep 2
kill -INT "$recorder"
wait "$recorder"
trap - EXIT
ros2 bag info euroc_recorded > recorded-info.txt
cat recorded-info.txt
