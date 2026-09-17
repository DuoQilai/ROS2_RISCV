#!/usr/bin/env bash
# COM260 Kit course; Gazebo and RViz run on the x86 host.
set -Eeuo pipefail
trap 'printf "Error: setup failed at line %s (exit %s).\n" "$LINENO" "$?" >&2' ERR
die() { printf 'Error: %s\n' "$*" >&2; exit 1; }

root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
workspace="$HOME/ros2_course_com260_ws"
env_dir="$HOME/.config/ros2-course-com260"
packages=(lifecycle_demo_cpp robot_sim_demo)
dependencies=(
  build-essential cmake gdb clangd python3-colcon-common-extensions
  python3-pytest rsync
  ros-humble-ros-base ros-humble-demo-nodes-cpp
  ros-humble-rmw-cyclonedds-cpp ros-humble-teleop-twist-keyboard
  ros-humble-ament-cmake-python ros-humble-ament-cmake-pytest
  ros-humble-rclcpp-lifecycle ros-humble-geometry-msgs
  ros-humble-sensor-msgs ros-humble-rosgraph-msgs ros-humble-nav-msgs
)
mode=${1:---help}
case "$mode" in
  --dry-run|--install-deps|--build|--build-ch02) ;;
  --help)
    printf '%s\n' 'Usage: bash setup_course_k3_com260_kit.sh --dry-run|--install-deps|--build|--build-ch02' \
      '--dry-run       Resolve chapter 1-2 dependencies without installing them.' \
      '--install-deps  Install chapter 1-2 dependencies; sudo may ask for your password.' \
      '--build         Build/test the two chapter 1 packages in ~/ros2_course_com260_ws.' \
      '--build-ch02    Build the two chapter 2 packages in the same course workspace.'
    exit 0 ;;
  *) printf 'Unknown option: %s\n' "$mode" >&2; exit 2 ;;
esac
[[ $# -eq 1 ]] || die 'Expected exactly one option; see --help.'
[[ $(uname -s) == Linux && $(uname -m) == riscv64 ]] || die 'Run this installer on the COM260 riscv64 Linux board.'
[[ $EUID -ne 0 ]] || die 'Run as the normal course user; --install-deps requests sudo when needed.'
[[ -r /etc/os-release ]] || die 'Cannot read /etc/os-release.'
source /etc/os-release
[[ ${ID:-} == bianbu ]] || die 'Bianbu is required.'
dpkg --compare-versions "${VERSION_ID:-0}" ge 4.0.1 || die 'Bianbu 4.0.1 or later is required.'
[[ -r /proc/device-tree/model ]] || die 'Cannot read the board model from /proc/device-tree/model.'
model=$(tr -d '\0' </proc/device-tree/model)
[[ $model == *K3*Com260* ]] || die 'This installer requires a SpacemiT K3 COM260 board.'
printf 'MODEL=%s OS=%s ROS=humble WORKSPACE=%s\n' "$model" "$VERSION_ID" "$workspace"

if [[ $mode == --dry-run ]]; then
  apt-get --simulate --no-remove --no-install-recommends install "${dependencies[@]}"
  exit 0
fi
if [[ $mode == --install-deps ]]; then
  sudo apt-get update
  sudo apt-get install -y --no-remove --no-install-recommends "${dependencies[@]}"
  exit 0
fi

[[ -z ${ROS_DISTRO:-} || $ROS_DISTRO == humble ]] || die 'Open a clean terminal without another ROS distribution loaded.'
[[ -r /opt/ros/humble/setup.bash ]] || die 'ROS 2 Humble is missing; run --install-deps first.'
for command in python3 cmake g++; do
  command -v "$command" >/dev/null || die "Missing $command; run --install-deps first."
done
python3 -c 'import colcon_core' || die 'The colcon Python module is missing; run --install-deps first.'
chapter=01
source_root="$root/src_k3_pico_itx"
if [[ $mode == --build-ch02 ]]; then
  chapter=02
  packages=(hello_pkg_cpp name_demo_cpp)
  source_root="$root/src_k3_com260_kit"
fi
# Chapter 1 packages are shared unchanged with the Pico edition.
sources=()
for package in "${packages[@]}"; do
  [[ -f "$source_root/$package/package.xml" ]] || die "Missing package source: $source_root/$package; sync the chapter files first."
  sources+=("$source_root/$package")
done
[[ ! -e $workspace || -f $workspace/.com260-ch01-workspace ]] || die "Refusing to reuse an unmarked workspace: $workspace"
mkdir -p "$workspace"
touch "$workspace/.com260-ch01-workspace"
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$workspace"
python3 -m colcon list --base-paths "${sources[@]}"
python3 -m colcon build --base-paths "${sources[@]}" --packages-select "${packages[@]}" \
  --executor sequential --symlink-install --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
if [[ $chapter == 01 ]]; then
  python3 -m colcon test --base-paths "${sources[@]}" --packages-select "${packages[@]}" \
    --event-handlers console_direct+ --return-code-on-test-failure
  python3 -m colcon test-result --verbose
fi
mkdir -p "$env_dir"
install -m 0755 "$root/course_support/k3_com260_kit/ide/gdb.bash" "$env_dir/gdb.bash"
cat >"$env_dir/env.bash" <<'EOF'
source /opt/ros/humble/setup.bash
source "$HOME/ros2_course_com260_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
EOF
set +u
source "$env_dir/env.bash"
set -u
for package in "${packages[@]}"; do ros2 pkg prefix "$package"; done
if [[ $chapter == 01 ]]; then
  printf 'COM260_CH01_BUILD_TEST_EXIT=0\n'
else
  printf 'COM260_CH02_BUILD_EXIT=0\n'
fi
printf 'Environment: source %s/env.bash\n' "$env_dir"
