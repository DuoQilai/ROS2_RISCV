#!/usr/bin/env bash
# Build an isolated Humble graph probe without replacing host Jazzy packages.
set -Eeuo pipefail

readonly IMAGE=localhost/ros2-course-humble:graph
readonly BASE_TAG=docker.io/library/ros:humble-ros-base-jammy
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
readonly SCRIPT_DIR
readonly CONTAINERFILE="$SCRIPT_DIR/course_support/k3/containers/Containerfile.humble"
mode=${1:-build}
[[ $# -le 1 ]] || { printf 'Only one mode is allowed.\n' >&2; exit 2; }
case "$mode" in
  build|--offline|--gazebo|--verify-gazebo|--dry-run|--verify) ;;
  --help|-h)
    printf 'Usage: bash setup_course_x86_humble_container.sh [--offline|--gazebo|--verify-gazebo|--dry-run|--verify]\n'
    printf 'Default: build and verify the rootless Humble graph-probe image.\n'
    printf 'Offline: use a previously verified and loaded official base image.\n'
    printf 'Gazebo: derive the Harmonic/RViz image after the Humble graph probe passes.\n'
    exit 0 ;;
  *) printf 'Unknown mode: %s\n' "$mode" >&2; exit 2 ;;
esac

# shellcheck disable=SC1091
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 && "$(uname -m)" == x86_64 ]]
[[ "$(systemd-detect-virt || true)" == none ]]
[[ "$(id -u)" -ne 0 ]]
[[ -f "$CONTAINERFILE" ]]
printf 'HOST=ubuntu-24.04 ARCH=x86_64 EXISTING_JAZZY=preserved\n'

if [[ "$mode" == --dry-run ]]; then
  printf '%s\n' \
    'Prerequisite, run interactively outside recording:' \
    'sudo apt-get update' \
    'sudo apt-get install -y --no-remove --no-install-recommends podman uidmap slirp4netns fuse-overlayfs' \
    "Pull $BASE_TAG; resolve and record its repository digest." \
    "Build $IMAGE from that digest using $CONTAINERFILE." \
    'Verify Ubuntu 22.04, Humble, CycloneDDS and the 24-byte Gid inside the container.' \
    'Runtime: rootless, host networking, no host filesystem mounts, no privileged mode.' \
    'CROSS_HOST_GRAPH=not_tested GAZEBO_GUI=not_tested'
  exit 0
fi

command -v podman >/dev/null || {
  printf 'Podman is missing. Install the prerequisites shown by --dry-run first.\n' >&2
  exit 1
}
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]]
printf 'PODMAN_ROOTLESS=true\n'

if [[ "$mode" == --gazebo || "$mode" == --verify-gazebo ]]; then
  sim_image=localhost/ros2-course-humble:gazebo
  sim_file="$SCRIPT_DIR/course_support/k3/containers/Containerfile.humble-gazebo"
  if [[ "$mode" == --gazebo ]]; then
    [[ "$(podman image inspect --format '{{index .Labels "org.ros2-riscv.component"}}' "$IMAGE")" == humble-graph-probe ]]
    if podman image exists "$sim_image"; then
      [[ "$(podman image inspect --format '{{index .Labels "org.ros2-riscv.component"}}' "$sim_image")" == humble-gazebo ]]
    fi
    sim_base_id=$(podman image inspect --format '{{.Id}}' "$IMAGE")
    sim_base_id=${sim_base_id#sha256:}
    [[ "$sim_base_id" =~ ^[0-9a-f]{64}$ ]]
    printf 'HUMBLE_GAZEBO_BASE_ID=sha256:%s\n' "$sim_base_id"
    podman build --pull=never --build-arg "ROS_BASE_REF=sha256:$sim_base_id" \
      --file "$sim_file" --tag "$sim_image" "$(dirname "$sim_file")"
  fi
  if ! podman image exists "$sim_image"; then
    printf 'Gazebo image missing: %s. Run bash setup_course_x86_humble_container.sh --gazebo first.\n' "$sim_image" >&2
    exit 1
  fi
  printf 'GAZEBO_CONTAINERFILE_SHA256=%s\n' "$(sha256sum "$sim_file" | cut -d ' ' -f 1)"
  printf 'GAZEBO_IMAGE_ID=%s\n' "$(podman image inspect --format '{{.Id}}' "$sim_image")"
  podman run --rm --network none --cap-drop all --security-opt no-new-privileges \
    "$sim_image" bash --noprofile --norc -c '
      set -eo pipefail
      source /etc/os-release
      [[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 && "$ROS_DISTRO" == humble && "$(uname -m)" == x86_64 ]]
      gz sim --versions | grep -E "^8[.]"
      for package in ros_gz_bridge ros_gz_sim ros_gz_image rviz2 robot_state_publisher rqt_graph rqt_console; do
        ros2 pkg prefix "$package" >/dev/null
      done
      dpkg-query -W gz-harmonic ros-humble-ros-gzharmonic ros-humble-rviz2
    '
  printf 'HUMBLE_GAZEBO_VERIFY_EXIT=0\n'
  printf 'GAZEBO_GUI=not_tested\n'
  exit 0
fi

if [[ "$mode" == build || "$mode" == --offline ]]; then
  if podman image exists "$IMAGE"; then
    [[ "$(podman image inspect --format '{{index .Labels "org.ros2-riscv.component"}}' "$IMAGE")" == humble-graph-probe ]]
  fi
  if [[ "$mode" == build ]]; then
    podman pull "$BASE_TAG"
    base_ref=$(podman image inspect --format '{{index .RepoDigests 0}}' "$BASE_TAG")
    [[ "$base_ref" =~ ^docker.io/library/ros@sha256:[0-9a-f]{64}$ ]]
  else
    # Archive import can regenerate manifest digests; pin the verified config ID.
    base_id=$(podman image inspect --format '{{.Id}}' "$BASE_TAG")
    base_id=${base_id#sha256:}
    [[ "$base_id" =~ ^[0-9a-f]{64}$ ]]
    base_ref="sha256:$base_id"
  fi
  [[ "$(podman image inspect --format '{{.Os}}/{{.Architecture}}' "$BASE_TAG")" == linux/amd64 ]]
  printf 'ROS_BASE_REF=%s\n' "$base_ref"
  podman build --pull=never --build-arg "ROS_BASE_REF=$base_ref" \
    --file "$CONTAINERFILE" --tag "$IMAGE" "$(dirname "$CONTAINERFILE")"
fi

printf 'CONTAINERFILE_SHA256=%s\n' "$(sha256sum "$CONTAINERFILE" | cut -d ' ' -f 1)"
printf 'IMAGE_ID=%s\n' "$(podman image inspect --format '{{.Id}}' "$IMAGE")"
podman run --rm --network host --cap-drop all --security-opt no-new-privileges \
  "$IMAGE" bash --noprofile --norc -c '
    set -eo pipefail
    source /etc/os-release
    [[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 && "$ROS_DISTRO" == humble ]]
    [[ "$(uname -m)" == x86_64 && "$RMW_IMPLEMENTATION" == rmw_cyclonedds_cpp ]]
    [[ "$ROS_DOMAIN_ID" == 0 && "$ROS_LOCALHOST_ONLY" == 0 ]]
    grep -Fx "char[24] data" /opt/ros/humble/share/rmw_dds_common/msg/Gid.msg
    ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null
    ros2 pkg prefix demo_nodes_cpp >/dev/null
    dpkg-query -W ros-humble-rmw-cyclonedds-cpp ros-humble-rmw-dds-common ros-humble-cyclonedds
    printf "CONTAINER=ubuntu-22.04 ARCH=x86_64 ROS_DISTRO=humble DOMAIN=0\n"
  '
printf 'HUMBLE_CONTAINER_VERIFY_EXIT=0\n'
printf 'CROSS_HOST_GRAPH=not_tested GAZEBO_GUI=not_tested\n'
