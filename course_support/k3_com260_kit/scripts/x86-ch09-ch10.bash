#!/usr/bin/env bash
# Isolated chapter 9/10 Humble, Harmonic, ros2_control and SLAM environment.
set -Eeuo pipefail
trap 'printf "Error: chapter 9/10 helper failed at line %s (exit %s).\n" "$LINENO" "$?" >&2' ERR
die() { printf 'Error: %s\n' "$*" >&2; exit 1; }
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
workspace="$HOME/ros2_course_com260_ch09_ch10_humble_ws"
image=localhost/ros2-course-com260:ch09-ch10
mode=${1:-}
[[ $(uname -s) == Linux && $(uname -m) == x86_64 && $EUID -ne 0 ]] || die 'Run as the normal x86 Linux user.'
command -v podman >/dev/null || die 'Podman is missing.'
[[ $(podman info --format '{{.Host.Security.Rootless}}') == true ]] || die 'Rootless Podman is required.'
case "$mode" in
  build-image)
    [[ $# -eq 1 ]] || die 'Usage: x86-ch09-ch10.bash build-image'
    podman build --pull=never -t "$image" -f "$root/course_support/k3_com260_kit/containers/Containerfile.ch09-ch10" "$root/course_support/k3_com260_kit/containers"
    exit 0 ;;
  build)
    [[ $# -eq 1 ]] || die 'Usage: x86-ch09-ch10.bash build'
    [[ ! -e $workspace || -f "$workspace/.com260-ch09-ch10-workspace" ]] || die "Refusing unmarked workspace: $workspace"
    mkdir -p "$workspace/output/maps" "$workspace/output/bags"
    touch "$workspace/.com260-ch09-ch10-workspace"
    podman run --rm --network none --cap-drop all --security-opt no-new-privileges \
      --volume "$root/src_k3_com260_kit:/course-src:ro" \
      --volume "$root/src_k3_pico_itx/robot_sim_demo:/shared-src/robot_sim_demo:ro" \
      --volume "$workspace:/workspace:rw" --workdir /workspace "$image" \
      bash --noprofile --norc -c 'set -e; source /opt/gz_ros2_control_ws/install/setup.bash; colcon build --base-paths /shared-src/robot_sim_demo /course-src/sim_demo /course-src/slam_lab /course-src/slam_sim_demo_ros2 /course-src/slam_teaching_cpp --packages-select robot_sim_demo sim_demo slam_lab slam_sim_demo_ros2 slam_teaching_cpp --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
    exit 0 ;;
  stop)
    [[ $# -eq 2 ]] || die 'Usage: x86-ch09-ch10.bash stop RUN_ID'
    exec bash "$root/course_support/k3_com260_kit/scripts/x86-gazebo.bash" stop "$2" ;;
  exec)
    [[ $# -ge 3 && "$2" =~ ^[[:alnum:]_-]+$ ]] || die 'Usage: x86-ch09-ch10.bash exec RUN_ID COMMAND [ARG ...]'
    run_id=$2; shift 2
    [[ $(podman inspect --format '{{index .Config.Labels "org.ros2-riscv.run-id"}}' "ros2-com260-$run_id") == "$run_id" ]] || die 'RUN_ID label mismatch.'
    exec podman exec -i "ros2-com260-$run_id" bash --noprofile --norc -c 'source /opt/gz_ros2_control_ws/install/setup.bash; source /workspace/install/setup.bash; exec "$@"' bash "$@" ;;
  start) ;;
  *) die 'Usage: x86-ch09-ch10.bash build-image | build | start RUN_ID | exec RUN_ID COMMAND [ARG ...] | stop RUN_ID' ;;
esac
[[ $# -eq 2 && "$2" =~ ^[[:alnum:]_-]+$ ]] || die 'Provide a simple alphanumeric RUN_ID.'
run_id=$2
[[ $(podman image inspect --format '{{index .Labels "org.ros2-riscv.component"}}' "$image") == humble-gazebo-slam-control ]] || die 'Build the chapter 9/10 image first.'
[[ -r "$workspace/install/setup.bash" ]] || die 'Build the chapter 9/10 workspace first.'
[[ -z $(podman ps --filter label=org.ros2-riscv.component=chapter-gazebo --format '{{.Names}}') ]] || die 'Stop the other course simulation before starting this run.'
auth_file="/run/user/$(id -u)/gdm/Xauthority"
[[ -r "$auth_file" && -S /tmp/.X11-unix/X1 ]] || die 'Log in to the x86 X11 :1 desktop first.'
DISPLAY=:1 XAUTHORITY="$auth_file" xdpyinfo >/dev/null || die 'Cannot access the x86 desktop.'
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-'<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'}
podman run --detach --rm --name "ros2-com260-$run_id" --network host \
  --cap-drop all --security-opt no-new-privileges \
  --label org.ros2-riscv.component=chapter-gazebo --label "org.ros2-riscv.run-id=$run_id" \
  --env DISPLAY=:1 --env XAUTHORITY=/tmp/course.xauth --env CYCLONEDDS_URI \
  --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" --env ROS_LOCALHOST_ONLY=0 \
  --env QT_X11_NO_MITSHM=1 --env LIBGL_ALWAYS_SOFTWARE=1 \
  --env QT_FONT_DPI=96 --env QT_ENABLE_HIGHDPI_SCALING=0 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:ro --volume "$auth_file:/tmp/course.xauth:ro" \
  --volume "$workspace:/workspace:rw" --workdir /workspace "$image" \
  timeout --signal=TERM --kill-after=5s 7200s sleep infinity >/dev/null
printf 'CH09_CH10_STARTED RUN_ID=%s\n' "$run_id"
