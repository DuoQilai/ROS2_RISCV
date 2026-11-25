#!/usr/bin/env bash
set -Eeuo pipefail
trap 'printf "Error: chapter 22/23 command failed at line %s (exit %s).\n" "$LINENO" "$?" >&2' ERR
die() { printf 'Error: %s\n' "$*" >&2; exit 1; }
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
workspace=${ROS2_COM260_WORKSPACE:-"$HOME/ros2_course_com260_ch22_ch23_humble_ws"}
image=localhost/ros2-course-com260:ch22-ch23
mode=${1:---help}
if [[ $mode == --help ]]; then
  echo 'Usage: x86-ch22-ch23.bash build-image | build | offline COMMAND [ARG ...] | start-camera RUN_ID [ARG ...] | start-slam RUN_ID [ARG ...] | start-nav RUN_ID [ARG ...] | start-explore RUN_ID [ARG ...] | start-project RUN_ID [ARG ...] | start-fleet RUN_ID | start-replay RUN_ID BAG_PATH | exec RUN_ID COMMAND [ARG ...] | stop RUN_ID'
  exit 0
fi
[[ $(uname -s) == Linux && $(uname -m) == x86_64 && $EUID -ne 0 ]] || die 'Run as the normal x86 Linux course user.'
command -v podman >/dev/null || die 'Prepare rootless Podman as described in chapter 0.'
[[ $(podman info --format '{{.Host.Security.Rootless}}') == true ]] || die 'Rootless Podman is required.'
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-222}
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-'<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'}
case "$mode" in
  build-image)
    [[ $# == 1 ]] || die 'Usage: x86-ch22-ch23.bash build-image'
    podman image exists localhost/ros2-course-com260:ch09-ch10 || die 'Build the chapter 9/10 image first.'
    podman image exists localhost/ros2-course-com260:ch20-ch21 || die 'Build the chapter 20/21 image first.'
    exec podman build -t "$image" -f "$root/course_support/k3_com260_kit/containers/Containerfile.ch22-ch23" "$root/course_support/k3_com260_kit/containers" ;;
  build)
    [[ $# == 1 ]] || die 'Usage: x86-ch22-ch23.bash build'
    [[ ! -e $workspace || -f "$workspace/.com260-ch22-ch23" ]] || die "Refusing unmarked workspace: $workspace"
    mkdir -p "$workspace"; touch "$workspace/.com260-ch22-ch23"
    podman image exists "$image" || die 'Run build-image first.'
    podman run --rm --network none --cap-drop all --security-opt no-new-privileges \
      --volume "$root/src_k3_com260_kit:/course/src_k3_com260_kit:ro" \
      --volume "$root/src_k3_pico_itx/robot_sim_demo:/course/robot_sim_demo:ro" \
      --volume "$workspace:/workspace:rw" --workdir /workspace "$image" \
      bash -c 'set -e; source /opt/ros/humble/setup.bash
        sources=(/course/robot_sim_demo)
        for package in "$@"; do sources+=("/course/src_k3_com260_kit/$package"); done
        colcon build --base-paths "${sources[@]}" --packages-select "$@" robot_sim_demo \
          --executor sequential --event-handlers console_direct+ \
          --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
        colcon test --base-paths "${sources[@]}" --packages-select "$@" \
          --event-handlers console_direct+ --return-code-on-test-failure
        colcon test-result --verbose' bash realsense_lab sensor_fusion_cpp slam_bringup_lab nav_bringup_lab \
          slam_sim_demo_ros2 param_demo_cpp slam_lab auto_explorer map_tools_cpp navigation_console fleet_scheduler
    exit 0 ;;
  offline)
    [[ $# -ge 2 && -r "$workspace/install/setup.bash" ]] || die 'Build first, then use: offline COMMAND [ARG ...]'
    shift
    mkdir -p "$HOME/maps"
    exec podman run --rm --network none --cap-drop all --security-opt no-new-privileges \
      --volume "$workspace:/workspace:rw" --volume "$HOME/maps:/maps:rw" --workdir /workspace "$image" \
      bash -c 'set -e; source /workspace/install/setup.bash; exec "$@"' bash "$@" ;;
  stop)
    [[ $# == 2 ]] || die 'Usage: x86-ch22-ch23.bash stop RUN_ID'
    exec bash "$root/course_support/k3_com260_kit/scripts/x86-gazebo.bash" stop "$2" ;;
  exec)
    [[ $# -ge 3 && "$2" =~ ^[[:alnum:]_-]+$ ]] || die 'Usage: x86-ch22-ch23.bash exec RUN_ID COMMAND [ARG ...]'
    run_id=$2; shift 2
    [[ $(podman inspect --format '{{index .Config.Labels "org.ros2-riscv.run-id"}}' "ros2-com260-$run_id") == "$run_id" ]] || die 'RUN_ID label mismatch.'
    terminal=()
    if [[ -t 0 && -t 1 ]]; then terminal=(-it); fi
    exec podman exec "${terminal[@]}" "ros2-com260-$run_id" bash -c 'set -e; source /workspace/install/setup.bash; source /opt/rviz-close-fix/local_setup.bash; exec "$@"' bash "$@" ;;
  start-camera|start-slam|start-nav|start-explore|start-project|start-fleet|start-replay) ;;
  *) die 'Unknown mode; see --help.' ;;
esac
[[ $# -ge 2 && "$2" =~ ^[[:alnum:]_-]+$ ]] || die 'A valid RUN_ID is required.'
run_id=$2
shift 2
[[ -r "$workspace/install/setup.bash" ]] || die 'Run build first.'
[[ -z $(podman ps --filter label=org.ros2-riscv.component=chapter-gazebo --format '{{.Names}}') ]] || die 'Stop the other course simulation first.'
gui=true; rviz=true
for argument in "$@"; do
  case "$argument" in gui:=*) gui=${argument#gui:=} ;; use_rviz:=*) rviz=${argument#use_rviz:=} ;; esac
done
display_args=()
if [[ ! $mode =~ ^start-(slam|nav|explore|project)$ || $gui != false || $rviz != false ]]; then
  display=:1
  auth_file="/run/user/$(id -u)/gdm/Xauthority"
  [[ -r $auth_file ]] || die 'Log in to the x86 X11 desktop first.'
  DISPLAY="$display" XAUTHORITY="$auth_file" xdpyinfo >/dev/null || die "Cannot access the x86 display $display."
  display_args=(--env DISPLAY="$display" --env XAUTHORITY=/tmp/course.xauth
    --volume /tmp/.X11-unix:/tmp/.X11-unix:ro --volume "$auth_file:/tmp/course.xauth:ro")
fi
mkdir -p "$HOME/maps"
render=(--env LIBGL_ALWAYS_SOFTWARE=1)
if [[ -r /dev/dri/renderD128 && -w /dev/dri/renderD128 ]]; then
  render=(--device /dev/dri --env LIBGL_ALWAYS_SOFTWARE=0)
fi
podman run --detach --rm --name "ros2-com260-$run_id" --network host \
  --cap-drop all --security-opt no-new-privileges \
  --label org.ros2-riscv.component=chapter-gazebo --label "org.ros2-riscv.run-id=$run_id" \
  "${display_args[@]}" --env CYCLONEDDS_URI \
  --env RMW_IMPLEMENTATION --env ROS_DOMAIN_ID --env ROS_LOCALHOST_ONLY \
  --env QT_X11_NO_MITSHM=1 "${render[@]}" \
  --volume "$root/course_support/k3_com260_kit/launch:/course-launch:ro" \
  --volume "$root/course_support/k3_com260_kit/config:/course-config:ro" \
  --volume "$root/course_support/k3_com260_kit/rviz:/course-rviz:ro" \
  --volume "$workspace:/workspace:rw" --volume "$HOME/maps:/maps:rw" --workdir /workspace "$image" \
  bash -c 'set -e; source /workspace/install/setup.bash; source /opt/rviz-close-fix/local_setup.bash
    run_id=$1; mode=$2; shift 2
    if [[ $mode == start-fleet ]]; then
      cd /course-config/fleet
      export GZ_PARTITION="$run_id"
      exec timeout --signal=INT --kill-after=10s 7200s ros2 launch /course-launch/multi-robot.launch.py
    fi
    if [[ $mode == start-replay ]]; then
      bag=$1; shift
      exec timeout --signal=INT --kill-after=10s 7200s ros2 launch sensor_fusion_cpp replay.launch.py bag:="$bag" "$@"
    fi
    case "$mode" in
      start-camera) package=realsense_lab; launch=realsense.launch.py ;;
      start-slam) package=slam_bringup_lab; launch=slam_bringup.launch.py ;;
      start-nav) package=nav_bringup_lab; launch=nav_bringup.launch.py ;;
      start-explore) package=auto_explorer; launch=exploration.launch.py ;;
      start-project) package=navigation_console; launch=project_navigation.launch.py ;;
    esac
    exec timeout --signal=INT --kill-after=10s 7200s ros2 launch "$package" "$launch" gz_partition:="$run_id" "$@"' bash "$run_id" "$mode" "$@"
