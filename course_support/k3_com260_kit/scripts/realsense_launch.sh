#!/usr/bin/env bash
set -euo pipefail
script=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/x86-ch22-ch23.bash
mode=${1:-rgbd}
case "$mode" in
  rgbd) args=(pointcloud:=false) ;;
  pointcloud) args=(pointcloud:=true) ;;
  highres) args=(pointcloud:=false profile:=1280x720x15) ;;
  *) echo 'Usage: realsense_launch.sh [rgbd|pointcloud|highres] [RUN_ID]' >&2; exit 2 ;;
esac
exec bash "$script" start-camera "${2:-ch12-$mode}" "${args[@]}"
