#!/usr/bin/env bash
# Run the chapter 6 Launch examples in the independent Humble/Nav2 image.
set -Eeuo pipefail
trap 'printf "Error: chapter 6 helper failed at line %s (exit %s).\n" "$LINENO" "$?" >&2' ERR
die() { printf 'Error: %s\n' "$*" >&2; exit 1; }
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
workspace="$HOME/ros2_course_com260_humble_ws"
image=localhost/ros2-course-com260:ch06-nav2
mode=${1:-}
[[ $(uname -s) == Linux && $(uname -m) == x86_64 ]] || die 'Run on the x86 Linux simulation host.'
command -v podman >/dev/null || die 'Podman is missing; prepare the chapter 0 environment first.'
[[ $(podman info --format '{{.Host.Security.Rootless}}') == true ]] || die 'Run as the normal rootless Podman user.'
case "$mode" in
  build-image)
    [[ $# -eq 1 ]] || die 'Usage: x86-ch06.bash build-image'
    podman build --pull=never -t "$image" \
      -f "$root/course_support/k3_com260_kit/containers/Containerfile.ch06-nav2" \
      "$root/course_support/k3_com260_kit/containers"
    exit 0 ;;
  build)
    [[ $# -eq 1 ]] || die 'Usage: x86-ch06.bash build'
    [[ -f "$workspace/.com260-ch01-workspace" ]] || die 'Build the chapter 1 simulation workspace first.'
    podman run --rm --network none --cap-drop all --security-opt no-new-privileges \
      --volume "$root/src_k3_com260_kit/param_demo_cpp:/course-src/param_demo_cpp:ro" \
      --volume "$workspace:/workspace:rw" --workdir /workspace "$image" \
      bash --noprofile --norc -c 'set -e; colcon build --base-paths /course-src/param_demo_cpp --packages-select param_demo_cpp --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Debug'
    exit 0 ;;
  stop)
    [[ $# -eq 2 ]] || die 'Usage: x86-ch06.bash stop RUN_ID'
    exec bash "$root/course_support/k3_com260_kit/scripts/x86-gazebo.bash" stop "$2" ;;
  exec)
    [[ $# -ge 3 && "$2" =~ ^[[:alnum:]_-]+$ ]] || die 'Usage: x86-ch06.bash exec RUN_ID COMMAND [ARG ...]'
    run_id=$2
    shift 2
    [[ $(podman inspect --format '{{index .Config.Labels "org.ros2-riscv.run-id"}}' "ros2-com260-$run_id") == "$run_id" ]] || die 'RUN_ID label mismatch.'
    exec podman exec "ros2-com260-$run_id" bash --noprofile --norc -c \
      'source /workspace/install/setup.bash; exec "$@"' bash "$@" ;;
  start) ;;
  *) die 'Usage: x86-ch06.bash build-image | build | start RUN_ID LAUNCH_FILE [name:=value ...] | exec RUN_ID COMMAND [ARG ...] | stop RUN_ID' ;;
esac
[[ $# -ge 3 && "$2" =~ ^[[:alnum:]_-]+$ ]] || die 'Provide a RUN_ID and one chapter 6 launch filename.'
run_id=$2
launch_file=$3
shift 3
case "$launch_file" in
  demo.launch.py|param_with_yaml.launch.py|combined_sim_nav.launch.py|gazebo_ch06.launch.py) ;;
  *) die 'Unsupported chapter 6 launch filename.' ;;
esac
[[ $(podman image inspect --format '{{index .Labels "org.ros2-riscv.component"}}' "$image") == humble-gazebo-nav2 ]] || die 'Build the chapter 6 image first.'
[[ -r "$workspace/install/param_demo_cpp/share/param_demo_cpp/launch/$launch_file" ]] || die 'Build the chapter 6 package first.'
[[ -z $(podman ps --filter label=org.ros2-riscv.component=chapter-gazebo --format '{{.Names}}') ]] || die 'Stop the other course simulation before starting this run.'
auth_file="/run/user/$(id -u)/gdm/Xauthority"
[[ -r "$auth_file" && -S /tmp/.X11-unix/X1 ]] || die 'Log in to the x86 X11 :1 desktop first.'
DISPLAY=:1 XAUTHORITY="$auth_file" xdpyinfo >/dev/null || die 'Cannot access the x86 desktop.'
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-'<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'}
podman run --detach --rm --name "ros2-com260-$run_id" --network host \
  --cap-drop all --security-opt no-new-privileges \
  --label org.ros2-riscv.component=chapter-gazebo --label "org.ros2-riscv.run-id=$run_id" \
  --env DISPLAY=:1 --env XAUTHORITY=/tmp/course.xauth --env CYCLONEDDS_URI \
  --env QT_X11_NO_MITSHM=1 --env LIBGL_ALWAYS_SOFTWARE=1 \
  --env QT_FONT_DPI=96 --env QT_ENABLE_HIGHDPI_SCALING=0 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:ro --volume "$auth_file:/tmp/course.xauth:ro" \
  --volume "$workspace:/workspace:ro" --workdir /workspace "$image" \
  bash --noprofile --norc -c '
    source /workspace/install/setup.bash
    exec timeout --signal=INT --kill-after=5s 3600s ros2 launch param_demo_cpp "$@"
  ' bash "$launch_file" "$@" >/dev/null
printf 'CH06_LAUNCH_STARTED RUN_ID=%s FILE=%s\n' "$run_id" "$launch_file"
