#!/usr/bin/env bash
set -Eeuo pipefail
trap 'printf "Error: chapter 20/21 command failed at line %s (exit %s).\n" "$LINENO" "$?" >&2' ERR
die() { printf 'Error: %s\n' "$*" >&2; exit 1; }
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
workspace=${ROS2_COM260_WORKSPACE:-"$HOME/ros2_course_com260_ch20_ch21_humble_ws"}
image=localhost/ros2-course-com260:ch20-ch21
mode=${1:---help}
if [[ $mode == --help ]]; then
  echo 'Usage: x86-ch20-ch21.bash build-image | build | start-fleet RUN_ID | start-lab RUN_ID | start-camera RUN_ID | start-slam RUN_ID | start-lvi RUN_ID | start-rgbd RUN_ID | exec RUN_ID COMMAND [ARG ...] | stop RUN_ID'
  exit 0
fi
[[ $(uname -s) == Linux && $(uname -m) == x86_64 && $EUID -ne 0 ]] || die 'Run as the normal x86 Linux course user.'
command -v podman >/dev/null || die 'Prepare rootless Podman as described in chapter 0.'
[[ $(podman info --format '{{.Host.Security.Rootless}}') == true ]] || die 'Rootless Podman is required.'
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-'<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'}
case "$mode" in
  build-image)
    [[ $# == 1 ]] || die 'Usage: x86-ch20-ch21.bash build-image'
    podman image exists localhost/ros2-course-com260:ch16-ch17-nav2 || die 'Build the chapter 16/17 RViz-fixed image first.'
    exec podman build -t "$image" -f "$root/course_support/k3_com260_kit/containers/Containerfile.ch20-ch21" "$root/course_support/k3_com260_kit/containers" ;;
  build)
    [[ $# == 1 ]] || die 'Usage: x86-ch20-ch21.bash build'
    [[ ! -e $workspace || -f "$workspace/.com260-ch20-ch21" ]] || die "Refusing unmarked workspace: $workspace"
    mkdir -p "$workspace"; touch "$workspace/.com260-ch20-ch21"
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
        colcon test-result --verbose' bash recovery_demo_cpp behavior_tree_demo_cpp multi_robot_scheduler \
          camera_geometry camera_processor orb_feature_extractor direct_image_alignment rgbd_slam vins_mono \
          navigation_client_cpp navigation_lab_cpp param_demo_cpp
    exit 0 ;;
  stop)
    [[ $# == 2 ]] || die 'Usage: x86-ch20-ch21.bash stop RUN_ID'
    exec bash "$root/course_support/k3_com260_kit/scripts/x86-gazebo.bash" stop "$2" ;;
  exec)
    [[ $# -ge 3 && "$2" =~ ^[[:alnum:]_-]+$ ]] || die 'Usage: x86-ch20-ch21.bash exec RUN_ID COMMAND [ARG ...]'
    run_id=$2; shift 2
    [[ $(podman inspect --format '{{index .Config.Labels "org.ros2-riscv.run-id"}}' "ros2-com260-$run_id") == "$run_id" ]] || die 'RUN_ID label mismatch.'
    exec podman exec "ros2-com260-$run_id" bash -c 'set -e; source /workspace/install/setup.bash; source /opt/rviz-close-fix/local_setup.bash; exec "$@"' bash "$@" ;;
  start-fleet|start-lab|start-camera|start-slam|start-lvi|start-rgbd) ;;
  *) die 'Unknown mode; see --help.' ;;
esac
[[ $# == 2 && "$2" =~ ^[[:alnum:]_-]+$ ]] || die 'A valid RUN_ID is required.'
run_id=$2
[[ -r "$workspace/install/setup.bash" ]] || die 'Run build first.'
[[ -z $(podman ps --filter label=org.ros2-riscv.component=chapter-gazebo --format '{{.Names}}') ]] || die 'Stop the other course simulation first.'
display=:1
auth_file="/run/user/$(id -u)/gdm/Xauthority"
if [[ $mode == start-lvi && -n ${DISPLAY:-} && -n ${XAUTHORITY:-} ]]; then
  display=$DISPLAY
  auth_file=$XAUTHORITY
fi
[[ -r $auth_file ]] || die 'Log in to the x86 X11 desktop first.'
DISPLAY="$display" XAUTHORITY="$auth_file" xdpyinfo >/dev/null || die "Cannot access the x86 display $display."
podman run --detach --rm --name "ros2-com260-$run_id" --network host \
  --cap-drop all --security-opt no-new-privileges \
  --label org.ros2-riscv.component=chapter-gazebo --label "org.ros2-riscv.run-id=$run_id" \
  --env DISPLAY="$display" --env XAUTHORITY=/tmp/course.xauth --env CYCLONEDDS_URI \
  --env RMW_IMPLEMENTATION --env ROS_DOMAIN_ID --env ROS_LOCALHOST_ONLY \
  --env QT_X11_NO_MITSHM=1 --env LIBGL_ALWAYS_SOFTWARE=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:ro --volume "$auth_file:/tmp/course.xauth:ro" \
  --volume "$root/course_support/k3_com260_kit/launch:/course-launch:ro" \
  --volume "$root/course_support/k3_com260_kit/config:/course-config:ro" \
  --volume "$root/course_support/k3_com260_kit/rviz:/course-rviz:ro" \
  --volume "$workspace:/workspace:rw" --workdir /workspace "$image" \
  bash -c 'set -e; source /workspace/install/setup.bash; source /opt/rviz-close-fix/local_setup.bash
    if [[ $2 == start-fleet ]]; then
      cd /course-config/fleet
      export GZ_PARTITION="$1"
      exec timeout --signal=INT --kill-after=10s 7200s ros2 launch /course-launch/multi-robot.launch.py
    fi
    if [[ $2 == start-rgbd ]]; then
      exec timeout --signal=INT --kill-after=10s 7200s ros2 launch /course-launch/rgbd-slam.launch.py gz_partition:="$1"
    fi
    if [[ $2 == start-slam || $2 == start-lvi ]]; then
      config=visual_slam.rviz
      if [[ $2 == start-lvi ]]; then config=lvi-sam.rviz; fi
      exec timeout --signal=INT --kill-after=10s 7200s ros2 run rviz2 rviz2 -d "/course-rviz/$config"
    fi
    if [[ $2 == start-lab ]]; then
      exec timeout --signal=INT --kill-after=10s 7200s ros2 launch /course-launch/planning-comparison.launch.py gz_partition:="$1" scene:=navigation_lab
    fi
    exec timeout --signal=INT --kill-after=10s 7200s ros2 launch robot_sim_demo gazebo2.launch.py gz_partition:="$1" rviz:=false drive:=true' bash "$run_id" "$mode"
