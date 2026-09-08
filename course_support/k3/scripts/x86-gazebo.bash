#!/usr/bin/env bash
# Run on the x86 host; keep Humble build outputs separate from native Jazzy.
set -Eeuo pipefail
mode=${1:?build, start, observe-lifecycle or stop required}
run_id=${2:-}
case "$mode" in build) ;; start|observe-lifecycle|stop) [[ "$run_id" =~ ^[[:alnum:]_-]+$ ]] || exit 2 ;; *) exit 2 ;; esac
# shellcheck disable=SC1091
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 && "$(uname -m)" == x86_64 ]]
[[ "$(systemd-detect-virt || true)" == none ]]
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]]
image=localhost/ros2-course-humble:gazebo
workspace="$HOME/ros2_course_humble_ws"
name="ros2-course-$run_id"

if [[ "$mode" == observe-lifecycle ]]; then
  [[ "$(podman inspect --format '{{index .Config.Labels "org.ros2-riscv.run-id"}}' "$name")" == "$run_id" ]]
  podman exec "$name" bash --noprofile --norc -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    printf "X86_RUN_ID=%s ROS=%s RMW=%s DOMAIN=%s\n" "$1" "$ROS_DISTRO" "$RMW_IMPLEMENTATION" "$ROS_DOMAIN_ID"
    twist=$(timeout 90s ros2 topic echo /cmd_vel geometry_msgs/msg/Twist --once)
    printf "%s\n" "$twist"
    printf "%s\n" "$twist" | grep -Fxq "  x: 0.1"
    printf "X86_CMD_VEL_LINEAR_X=0.1\n"
    state=$(timeout 10s ros2 lifecycle get /hello_ros2_lifecycle)
    [[ "$state" == "active [3]" ]]
    printf "X86_LIFECYCLE_STATE=%s\n" "$state"
    nodes=$(timeout 10s ros2 node list --no-daemon)
    printf "%s\n" "$nodes" | grep -Fxq /hello_ros2_lifecycle
    printf "X86_K3_NODE_VISIBLE=true\n"
  ' bash "$run_id"
  exit 0
fi

if [[ "$mode" == stop ]]; then
  [[ "$(podman inspect --format '{{index .Config.Labels "org.ros2-riscv.run-id"}}' "$name")" == "$run_id" ]]
  podman stop --time 10 "$name" >/dev/null
  printf 'X86_GAZEBO_STOP_EXIT=0 RUN_ID=%s\n' "$run_id"
  exit 0
fi
[[ "$(podman image inspect --format '{{index .Labels "org.ros2-riscv.component"}}' "$image")" == humble-gazebo ]]
printf 'GAZEBO_IMAGE_ID=%s\n' "$(podman image inspect --format '{{.Id}}' "$image")"
if [[ "$mode" == build ]]; then
  [[ -f "$HOME/ROS2_RISCV/src/robot_sim_demo/package.xml" ]]
  mkdir -p "$workspace"
  podman run --rm --network none --cap-drop all --security-opt no-new-privileges \
    --volume "$HOME/ROS2_RISCV/src/robot_sim_demo:/course-src/robot_sim_demo:ro" \
    --volume "$workspace:/workspace:rw" --workdir /workspace \
    "$image" bash --noprofile --norc -c '
      set -eo pipefail
      colcon build --base-paths /course-src/robot_sim_demo --event-handlers console_direct+
      source install/setup.bash
      ros2 pkg prefix robot_sim_demo >/dev/null
    '
  printf 'HUMBLE_SIM_BUILD_EXIT=0\n'
  exit 0
fi

[[ -f "$workspace/install/setup.bash" ]]
[[ -z "$(podman ps --filter label=org.ros2-riscv.component=chapter-gazebo --format '{{.ID}}')" ]]
if podman container exists "$name"; then exit 1; fi
auth_file="/run/user/$(id -u)/gdm/Xauthority"
[[ -r "$auth_file" && -S /tmp/.X11-unix/X1 ]]
DISPLAY=:1 XAUTHORITY="$auth_file" xdpyinfo >/dev/null
podman run --detach --rm --name "$name" --network host \
  --cap-drop all --security-opt no-new-privileges --hostname "$(hostname)" \
  --label org.ros2-riscv.component=chapter-gazebo --label "org.ros2-riscv.run-id=$run_id" \
  --env DISPLAY=:1 --env XAUTHORITY=/tmp/course.xauth \
  --env QT_X11_NO_MITSHM=1 --env LIBGL_ALWAYS_SOFTWARE=1 \
  --env QT_FONT_DPI=96 --env QT_ENABLE_HIGHDPI_SCALING=0 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:ro --volume "$auth_file:/tmp/course.xauth:ro" \
  --volume "$workspace:/workspace:ro" --workdir /workspace \
  "$image" bash --noprofile --norc -c '
    source /workspace/install/setup.bash
    exec timeout --signal=INT --kill-after=5s 3600s ros2 launch robot_sim_demo \
      gazebo2.launch.py gui:=true rviz:=true drive:=false gz_partition:="$1"
  ' bash "$run_id" >/dev/null
printf 'X86_GAZEBO_STARTED RUN_ID=%s RENDERER=software DRIVE=false\n' "$run_id"
