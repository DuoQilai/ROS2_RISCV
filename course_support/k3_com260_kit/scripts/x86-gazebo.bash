#!/usr/bin/env bash
# Run on the x86 host; keep Humble build outputs separate from native Jazzy.
set -Eeuo pipefail
trap 'printf "Error: x86 course helper failed at line %s (exit %s).\n" "$LINENO" "$?" >&2' ERR
die() { printf 'Error: %s\n' "$*" >&2; exit 1; }
mode=${1:-}
run_id=${2:-}
case "$mode" in
  build) [[ $# -eq 1 ]] || die 'Usage: x86-gazebo.bash build' ;;
  start|observe-lifecycle|stop)
    [[ $# -eq 2 && "$run_id" =~ ^[[:alnum:]_-]+$ ]] || die 'Provide one RUN_ID containing only letters, digits, underscores or hyphens.' ;;
  *) printf 'Usage: x86-gazebo.bash build | start RUN_ID | observe-lifecycle RUN_ID | stop RUN_ID\n' >&2; exit 2 ;;
esac
[[ $(uname -s) == Linux && $(uname -m) == x86_64 ]] || die 'Run this helper on the x86_64 Linux simulation host.'
# shellcheck disable=SC1091
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 ]] || die 'This helper requires the Ubuntu 24.04 simulation host.'
[[ "$(systemd-detect-virt || true)" == none ]] || die 'Run on the simulation host, outside a container or virtual machine.'
command -v podman >/dev/null || die 'Podman is missing; prepare the x86 course environment first.'
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] || die 'Run Podman as the normal rootless course user.'
image=localhost/ros2-course-humble:gazebo
workspace="$HOME/ros2_course_com260_humble_ws"
name="ros2-com260-$run_id"

if [[ "$mode" == stop || "$mode" == observe-lifecycle ]]; then
  if podman container exists "$name"; then
    [[ "$(podman inspect --format '{{index .Config.Labels "org.ros2-riscv.run-id"}}' "$name")" == "$run_id" ]] || die "RUN_ID label mismatch; refusing to use container $name."
  else
    status=$?
    [[ $status -eq 1 ]] || die "Cannot query container $name (Podman exit $status)."
    [[ "$mode" == stop ]] || die "Container $name does not exist; start this run before observing it."
    printf 'X86_GAZEBO_ALREADY_ABSENT RUN_ID=%s; the container may have stopped or reached its one-hour limit.\n' "$run_id"
    exit 0
  fi
fi

if [[ "$mode" == observe-lifecycle ]]; then
  podman exec "$name" bash --noprofile --norc -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    printf "X86_RUN_ID=%s ROS=%s RMW=%s DOMAIN=%s\n" "$1" "$ROS_DISTRO" "$RMW_IMPLEMENTATION" "$ROS_DOMAIN_ID"
    velocity=$(timeout 90s ros2 topic echo /cmd_vel geometry_msgs/msg/Twist --once --field linear.x)
    printf "%s\n" "$velocity" | python3 -c "import math, sys, yaml; value = next(yaml.safe_load_all(sys.stdin), None); sys.exit(0 if isinstance(value, (int, float)) and math.isclose(value, 0.1, rel_tol=0.0, abs_tol=1e-9) else \"Expected /cmd_vel linear.x=0.1\")"
    printf "X86_CMD_VEL_LINEAR_X=0.1\n"
    state=$(timeout 25s ros2 lifecycle get /hello_ros2_lifecycle)
    [[ "$state" == "active [3]" ]] || { printf "Expected lifecycle state active [3]; got %s\n" "$state" >&2; exit 1; }
    printf "X86_LIFECYCLE_STATE=%s\n" "$state"
    nodes=$(timeout 25s ros2 node list --no-daemon --spin-time 10)
    printf "%s\n" "$nodes" | grep -Fxq /hello_ros2_lifecycle || { printf "COM260 lifecycle node is not visible.\n" >&2; exit 1; }
    printf "X86_COM260_NODE_VISIBLE=true\n"
  ' bash "$run_id"
  exit 0
fi

if [[ "$mode" == stop ]]; then
  podman stop --time 10 "$name" >/dev/null || die "Failed to stop container $name."
  printf 'X86_GAZEBO_STOP_EXIT=0 RUN_ID=%s\n' "$run_id"
  exit 0
fi
[[ "$(podman image inspect --format '{{index .Labels "org.ros2-riscv.component"}}' "$image")" == humble-gazebo ]] || die "Missing or mismatched course image: $image; prepare course_support/k3/containers/ first."
printf 'GAZEBO_IMAGE_ID=%s\n' "$(podman image inspect --format '{{.Id}}' "$image")"
if [[ "$mode" == build ]]; then
  [[ -f "$HOME/ROS2_RISCV_COM260/src_k3_pico_itx/robot_sim_demo/package.xml" ]] || die 'Missing robot_sim_demo source; sync the chapter files first.'
  [[ ! -e "$workspace" || -f "$workspace/.com260-ch01-workspace" ]] || die "Refusing to reuse an unmarked workspace: $workspace"
  mkdir -p "$workspace"
  touch "$workspace/.com260-ch01-workspace"
  podman run --rm --network none --cap-drop all --security-opt no-new-privileges \
    --volume "$HOME/ROS2_RISCV_COM260/src_k3_pico_itx/robot_sim_demo:/course-src/robot_sim_demo:ro" \
    --volume "$workspace:/workspace:rw" --workdir /workspace \
    "$image" bash --noprofile --norc -c '
      set -eo pipefail
      colcon build --base-paths /course-src/robot_sim_demo --event-handlers console_direct+
      colcon test --base-paths /course-src/robot_sim_demo --event-handlers console_direct+ --return-code-on-test-failure
      colcon test-result --verbose
      source install/setup.bash
      ros2 pkg prefix robot_sim_demo >/dev/null
    '
  printf 'HUMBLE_SIM_BUILD_EXIT=0\n'
  exit 0
fi

[[ -f "$workspace/install/setup.bash" ]] || die 'The simulation workspace is not built; run x86-gazebo.bash build first.'
active=$(podman ps --filter label=org.ros2-riscv.component=chapter-gazebo --format '{{.Names}}')
[[ -z "$active" ]] || die "Another course simulation is active: $active; finish its run before starting another."
if podman container exists "$name"; then
  die "Container $name already exists; use its original RUN_ID to inspect or stop it."
else
  status=$?
  [[ $status -eq 1 ]] || die "Cannot query container $name (Podman exit $status)."
fi
auth_file="/run/user/$(id -u)/gdm/Xauthority"
[[ -r "$auth_file" && -S /tmp/.X11-unix/X1 ]] || die 'The expected GDM X11 :1 session is unavailable; log in to the x86 desktop and check the session paths in chapter 0.'
command -v xdpyinfo >/dev/null || die 'xdpyinfo is missing; install x11-utils on the simulation host.'
DISPLAY=:1 XAUTHORITY="$auth_file" xdpyinfo >/dev/null || die 'Cannot access the X11 :1 session with the current GDM authority file.'
podman run --detach --rm --name "$name" --network host \
  --cap-drop all --security-opt no-new-privileges --hostname "$(hostname)" \
  --label org.ros2-riscv.component=chapter-gazebo --label "org.ros2-riscv.run-id=$run_id" \
  --env DISPLAY=:1 --env XAUTHORITY=/tmp/course.xauth \
  --env QT_X11_NO_MITSHM=1 --env LIBGL_ALWAYS_SOFTWARE=1 \
  --env QT_FONT_DPI=96 --env QT_ENABLE_HIGHDPI_SCALING=0 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:ro --volume "$auth_file:/tmp/course.xauth:ro" \
  --volume "$HOME/ROS2_RISCV_COM260/course_support/k3_com260_kit/rviz:/course-rviz:ro" \
  --volume "$workspace:/workspace:ro" --workdir /workspace \
  "$image" bash --noprofile --norc -c '
    source /workspace/install/setup.bash
    exec timeout --signal=INT --kill-after=5s 3600s ros2 launch robot_sim_demo \
      gazebo2.launch.py gui:=true rviz:=true drive:=false gz_partition:="$1" \
      rviz_config:=/course-rviz/museum.rviz
  ' bash "$run_id" >/dev/null
printf 'X86_GAZEBO_STARTED RUN_ID=%s RENDERER=software DRIVE=false\n' "$run_id"
