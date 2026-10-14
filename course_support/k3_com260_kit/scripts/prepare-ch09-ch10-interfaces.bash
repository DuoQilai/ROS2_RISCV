#!/usr/bin/env bash
# Build the Cartographer service interfaces used by board-side chapter 10 clients.
set -Eeuo pipefail
trap 'printf "Error: interface preparation failed at line %s (exit %s).\n" "$LINENO" "$?" >&2' ERR
die() { printf 'Error: %s\n' "$*" >&2; exit 1; }
[[ $(uname -s) == Linux && $(uname -m) == riscv64 && $EUID -ne 0 ]] || die 'Run as the normal COM260 riscv64 Linux user.'
[[ -r /opt/ros/humble/setup.bash ]] || die 'ROS 2 Humble is required.'
for tool in git cmake python3; do command -v "$tool" >/dev/null || die "Missing $tool."; done
workspace="$HOME/ros2_course_com260_ch09_ch10_deps_ws"
[[ ! -e $workspace || -f "$workspace/.com260-ch09-ch10-deps" ]] || die "Refusing unmarked workspace: $workspace"
mkdir -p "$workspace/src"
touch "$workspace/.com260-ch09-ch10-deps"
revision=43cd92bc8c4bf1a90f50eddbbbd724a899ceebe1
checkout="$workspace/src/cartographer_ros"
if [[ ! -d "$checkout/.git" ]]; then
  [[ ! -e $checkout ]] || die "Refusing existing non-Git source: $checkout"
  git init "$checkout"
  git -C "$checkout" remote add origin https://github.com/ros2/cartographer_ros.git
fi
[[ -z $(git -C "$checkout" status --porcelain) ]] || die 'Interface source has local changes; inspect it before building.'
if ! git -C "$checkout" rev-parse --verify HEAD >/dev/null 2>&1; then
  git -c http.lowSpeedLimit=1024 -c http.lowSpeedTime=60 -C "$checkout" fetch --depth 1 origin "$revision"
  git -C "$checkout" checkout --detach FETCH_HEAD
fi
[[ $(git -C "$checkout" rev-parse HEAD) == "$revision" ]] || die 'Interface source revision differs from the chapter pin.'
nav_revision=a097086719c88f781aa59788eca29ac6ca5e56db
nav_checkout="$workspace/src/navigation2"
if [[ ! -d "$nav_checkout/.git" ]]; then
  [[ ! -e $nav_checkout ]] || die "Refusing existing non-Git source: $nav_checkout"
  git init "$nav_checkout"
  git -C "$nav_checkout" remote add origin https://github.com/ros-navigation/navigation2.git
fi
[[ -z $(git -C "$nav_checkout" status --porcelain) ]] || die 'Nav2 interface source has local changes; inspect it before building.'
if ! git -C "$nav_checkout" rev-parse --verify HEAD >/dev/null 2>&1; then
  git -c http.lowSpeedLimit=1024 -c http.lowSpeedTime=60 -C "$nav_checkout" fetch --depth 1 origin "$nav_revision"
  git -C "$nav_checkout" checkout --detach FETCH_HEAD
fi
[[ $(git -C "$nav_checkout" rev-parse HEAD) == "$nav_revision" ]] || die 'Nav2 interface source revision differs from the chapter pin.'
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$workspace"
python3 -m colcon build --base-paths "$checkout/cartographer_ros_msgs" "$nav_checkout/nav2_common" "$nav_checkout/nav2_msgs" \
  --packages-select cartographer_ros_msgs nav2_common nav2_msgs --executor sequential --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=OFF
printf 'Interface underlay: source %s/install/setup.bash\n' "$workspace"
