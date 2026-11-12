#!/usr/bin/env bash
# Build and run the chapter 18/19 x86 Nav2 simulation workspace.
set -Eeuo pipefail
trap 'printf "Error: Nav2 teaching command failed at line %s (exit %s).\n" "$LINENO" "$?" >&2' ERR
die() { printf 'Error: %s\n' "$*" >&2; exit 1; }
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
image=localhost/ros2-course-com260:ch16-ch17-nav2
mode=${1:---help}
[[ $mode != --help ]] || {
  echo 'Usage: x86-ch18-ch19.bash build-image | build | start RUN_ID [PLANNER [CONTROLLER]] | start-lab RUN_ID | exec RUN_ID COMMAND [ARG ...] | stop RUN_ID'
  exit 0
}
[[ $(uname -s) == Linux && $EUID -ne 0 ]] || die 'Run as the normal Linux course user.'
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-'<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'}
[[ $(uname -m) == x86_64 ]] || die 'This mode requires the x86 simulation host.'
workspace="$HOME/ros2_course_com260_ch18_ch19_humble_ws"
command -v podman >/dev/null || die 'Prepare rootless Podman as described in chapter 0.'
[[ $(podman info --format '{{.Host.Security.Rootless}}') == true ]] || die 'Rootless Podman is required.'
case "$mode" in
  build-image)
    [[ $# == 1 ]] || die 'Usage: x86-ch18-ch19.bash build-image'
    bash "$root/course_support/k3_com260_kit/scripts/x86-ch16-ch17.bash" build-image
    exit 0 ;;
  build)
    [[ $# == 1 ]] || die 'Usage: x86-ch18-ch19.bash build'
    [[ ! -e $workspace || -f "$workspace/.com260-ch18-ch19" ]] || die "Refusing unmarked workspace: $workspace"
    mkdir -p "$workspace"
    touch "$workspace/.com260-ch18-ch19"
    podman image exists "$image" || die 'Build the chapter 18/19 image first: x86-ch18-ch19.bash build-image'
    podman run --rm --network none --cap-drop all --security-opt no-new-privileges \
      --volume "$root/src_k3_com260_kit:/course/src_k3_com260_kit:ro" \
      --volume "$root/src_k3_pico_itx/robot_sim_demo:/course/src_k3_pico_itx/robot_sim_demo:ro" \
      --volume "$workspace:/workspace:rw" --workdir /workspace "$image" \
      bash -c 'set -e; source /opt/ros/humble/setup.bash
        sources=(/course/src_k3_pico_itx/robot_sim_demo)
        for package in "$@"; do sources+=("/course/src_k3_com260_kit/$package"); done
        colcon build --base-paths "${sources[@]}" --packages-select "$@" robot_sim_demo \
          --executor sequential --event-handlers console_direct+ \
          --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
        colcon test --base-paths "${sources[@]}" \
          --packages-select grid_path_planner hybrid_astar_planner path_optimizer dwa_planner rpp_controller \
          --event-handlers console_direct+ --return-code-on-test-failure
        colcon test-result --verbose' bash costmap_demo grid_path_planner hybrid_astar_planner \
          path_optimizer dwa_planner rpp_controller global_planner_client navigation_client_cpp navigation_lab_cpp param_demo_cpp
    exit 0 ;;
  stop)
    [[ $# == 2 ]] || die 'Usage: x86-ch18-ch19.bash stop RUN_ID'
    bash "$root/course_support/k3_com260_kit/scripts/x86-gazebo.bash" stop "$2"
    # A host reboot can leave an auto-remove container in Created state.
    if podman container exists "ros2-com260-$2"; then
      podman rm "ros2-com260-$2" >/dev/null
    fi
    exit 0 ;;
  exec)
    [[ $# -ge 3 && "$2" =~ ^[[:alnum:]_-]+$ ]] || die 'Usage: x86-ch18-ch19.bash exec RUN_ID COMMAND [ARG ...]'
    run_id=$2; shift 2
    [[ $(podman inspect --format '{{index .Config.Labels "org.ros2-riscv.run-id"}}' "ros2-com260-$run_id") == "$run_id" ]] || die 'RUN_ID label mismatch.'
    exec podman exec "ros2-com260-$run_id" bash -c 'set -e; source /workspace/install/setup.bash; source /opt/rviz-close-fix/local_setup.bash; exec "$@"' bash "$@" ;;
  start|start-lab) ;;
  *) die 'Unknown mode; see --help.' ;;
esac
[[ $# -ge 2 && $# -le 4 && "$2" =~ ^[[:alnum:]_-]+$ ]] || die 'Usage: x86-ch18-ch19.bash start RUN_ID [PLANNER [CONTROLLER]]'
run_id=$2
scene=museum
if [[ $mode == start-lab ]]; then
  [[ $# == 2 ]] || die 'Usage: x86-ch18-ch19.bash start-lab RUN_ID'
  scene=navigation_lab
fi
planner=${3:-navfn}
controller=${4:-dwb}
case "$planner" in navfn|astar|smac2d|hybrid) ;; *) die 'Planner must be navfn, astar, smac2d or hybrid.' ;; esac
case "$controller" in dwb|rpp|mppi) ;; *) die 'Controller must be dwb, rpp or mppi.' ;; esac
[[ -r "$workspace/install/setup.bash" ]] || die 'Run build first.'
[[ -z $(podman ps --filter label=org.ros2-riscv.component=chapter-gazebo --format '{{.Names}}') ]] || die 'Stop the other course simulation first.'
auth_file="/run/user/$(id -u)/gdm/Xauthority"
[[ -r $auth_file && -S /tmp/.X11-unix/X1 ]] || die 'Log in to the x86 X11 :1 desktop first.'
DISPLAY=:1 XAUTHORITY="$auth_file" xdpyinfo >/dev/null || die 'Cannot access the x86 desktop.'
podman run --detach --rm --name "ros2-com260-$run_id" --network host \
  --cap-drop all --security-opt no-new-privileges \
  --label org.ros2-riscv.component=chapter-gazebo --label "org.ros2-riscv.run-id=$run_id" \
  --env DISPLAY=:1 --env XAUTHORITY=/tmp/course.xauth --env CYCLONEDDS_URI \
  --env RMW_IMPLEMENTATION --env ROS_DOMAIN_ID --env ROS_LOCALHOST_ONLY \
  --env QT_X11_NO_MITSHM=1 --env LIBGL_ALWAYS_SOFTWARE=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:ro --volume "$auth_file:/tmp/course.xauth:ro" \
  --volume "$root/course_support/k3_com260_kit/launch:/course-launch:ro" \
  --volume "$root/course_support/k3_com260_kit/config:/course-config:ro" \
  --volume "$workspace:/workspace:rw" --workdir /workspace "$image" \
  bash -c 'set -e; source /workspace/install/setup.bash; source /opt/rviz-close-fix/local_setup.bash; exec timeout --signal=INT --kill-after=10s 7200s ros2 launch /course-launch/planning-comparison.launch.py gz_partition:="$1" planner:="$2" controller:="$3" scene:="$4"' bash "$run_id" "$planner" "$controller" "$scene"
