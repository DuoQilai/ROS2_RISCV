#!/usr/bin/env bash
# RISC-V ROS2 机器人操作系统编程技术 - K3 Pico-ITX 课程环境安装器
# Target: SpacemiT K3 Pico-ITX + Bianbu >= 4.0.1 (riscv64) + ROS 2 Humble
# Gazebo and RViz2 run on the x86 host. Its OS, ROS/Gazebo versions and DDS
# connection settings must be confirmed by the K3/x86 validation contract.
# Uses the ROS 2 packages provided by the configured Bianbu apt repositories.

set -Eeuo pipefail
IFS=$'\n\t'

TARGET_BIANBU_OS_ID="bianbu"
TARGET_BIANBU_MIN_VERSION="4.0.1"
TARGET_ROS_DISTRO="humble"
PLATFORM_ID=""
PLATFORM_VERSION=""
ROSDEP_READY=false

COURSE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COURSE_WS="${ROS2_K3_COURSE_WS:-${HOME}/ros2_course_k3_ws}"
COURSE_SRC="${COURSE_ROOT}/src_k3_pico_itx"
ML_VENV="${ROS2_K3_COURSE_ML_VENV:-${HOME}/.venvs/ros2-course-ml}"
ENV_DIR="${XDG_CONFIG_HOME:-${HOME}/.config}/ros2-course-k3"
ENV_FILE="${ENV_DIR}/env.bash"

WITH_ML=false
WITH_HARDWARE=false
RUN_TESTS=false
VERIFY_ONLY=false
DRY_RUN=false
REFRESH_ENV=false

CURRENT_STEP="startup"
LOCK_FD=9

readonly BASHRC_BEGIN="# >>> ROS2 K3 course environment >>>"
readonly BASHRC_END="# <<< ROS2 K3 course environment <<<"

BASE_APT_PACKAGES=(
  ca-certificates
  cmake
  curl
  gcc
  g++
  gdb
  git
  make
  pkg-config
  procps
  python3-dev
  python3-pip
  python3-setuptools
  python3-venv
  python3-wheel
  rsync
  tar
  tmux
  unzip
  debianutils
  util-linux
  xz-utils
)

BEST_EFFORT_APT_PACKAGES=(
  python3-colcon-common-extensions
  python3-rosdep
  python3-matplotlib
  python3-numpy
  python3-opencv
  python3-pytest
  python3-pytest-cov
  python3-serial
  python3-yaml
  python3-requests
  python3-sklearn
  python3-scipy
)

REQUIRED_ROS_PACKAGES=(
  ros-humble-ros-base
  ros-humble-demo-nodes-cpp
  ros-humble-rmw-cyclonedds-cpp
  ros-humble-tf2-tools
  ros-humble-turtlesim
  ros-humble-teleop-twist-keyboard
)

BEST_EFFORT_ROS_PACKAGES=(
  ros-humble-demo-nodes-py
  ros-humble-joint-state-publisher-gui
  ros-humble-launch-testing
  ros-humble-launch-testing-ament-cmake
  ros-humble-nav2-bringup
  ros-humble-navigation2
  ros-humble-robot-localization
  ros-humble-robot-state-publisher
  ros-humble-ros-testing
  ros-humble-rosbag2
  ros-humble-rqt-graph
  ros-humble-rqt-image-view
  ros-humble-rtabmap-ros
  ros-humble-slam-toolbox
  ros-humble-tf-transformations
  ros-humble-xacro
)

HARDWARE_ROS_PACKAGES=(
  ros-humble-usb-cam
)

ML_PIP_PACKAGES=(
  "evo==1.31.1"
  "filterpy==1.4.5"
  "openai>=1.0,<3"
)

usage() {
  cat <<'EOF'
Usage: bash setup_course_k3.sh [options]

Target platform: SpacemiT K3 Pico-ITX with Bianbu 4.0.1 or newer on RISC-V
(riscv64), with ROS 2 Humble from the configured Bianbu apt repositories.
This is the board-side installer; Gazebo and RViz2 run on the x86 host. Confirm
its OS, runtime, ROS/Gazebo versions, RMW and DDS domain before cross-machine
validation.

Default action:
  Use the configured platform package repository to install ROS 2 Humble and
  base dependencies, synchronize the course into ~/ros2_course_k3_ws, build all
  ROS packages, configure ~/.bashrc, and verify.

Options:
  --with-ml             Install ML dependencies in an isolated venv
                        (wheels on riscv64 may need to compile)
  --with-hardware       Install camera / serial / fiducial dependencies
  --workspace PATH      Use a managed workspace other than ~/ros2_course_k3_ws
  --run-tests           Run colcon tests after a successful build
  --verify              Verify an existing installation without changing it
  --refresh-env         Regenerate the shell environment without reinstalling
  --dry-run             Print mutating commands without executing them
  --help                Show this help

Environment overrides:
  ROS2_K3_COURSE_WS, ROS2_K3_COURSE_ML_VENV
EOF
}

log_info() {
  printf '[INFO] %s\n' "$*"
}

log_ok() {
  printf '[ OK ] %s\n' "$*"
}

log_warn() {
  printf '[WARN] %s\n' "$*" >&2
}

die() {
  printf '[FAIL] %s\n' "$*" >&2
  exit 1
}

on_error() {
  local exit_code=$?
  local line_no=$1
  local command=$2
  printf '[FAIL] Step "%s" failed at line %s (exit=%s): %s\n' \
    "${CURRENT_STEP}" "${line_no}" "${exit_code}" "${command}" >&2
  exit "${exit_code}"
}

trap 'on_error "${LINENO}" "${BASH_COMMAND}"' ERR

print_command() {
  printf '[DRY-RUN]'
  printf ' %q' "$@"
  printf '\n'
}

run() {
  if [[ "${DRY_RUN}" == true ]]; then
    print_command "$@"
    return 0
  fi
  "$@"
}

run_best_effort() {
  if [[ "${DRY_RUN}" == true ]]; then
    print_command "$@"
    return 0
  fi
  if ! "$@"; then
    log_warn "Best-effort step failed, continuing: $*"
    return 0
  fi
}

parse_args() {
  while (($# > 0)); do
    case "$1" in
      --with-ml)
        WITH_ML=true
        ;;
      --with-hardware)
        WITH_HARDWARE=true
        ;;
      --with-carla|--carla-bridge-only|--carla-only|--skip-carla)
        die "The CARLA autonomous-driving part has been removed from this course; this option is no longer supported"
        ;;
      --all-profiles)
        WITH_ML=true
        WITH_HARDWARE=true
        ;;
      --workspace)
        (($# >= 2)) || die "--workspace requires a path"
        COURSE_WS="$2"
        shift
        ;;
      --run-tests)
        RUN_TESTS=true
        ;;
      --verify)
        VERIFY_ONLY=true
        ;;
      --refresh-env)
        REFRESH_ENV=true
        ;;
      --dry-run)
        DRY_RUN=true
        ;;
      --ros2-only)
        log_warn "--ros2-only is deprecated; the default profile has the same behavior"
        ;;
      --help|-h)
        usage
        exit 0
        ;;
      *)
        die "Unknown option: $1"
        ;;
    esac
    shift
  done

  COURSE_WS="${COURSE_WS/#\~/${HOME}}"
  [[ "${COURSE_WS}" == /* ]] || die "Workspace path must be absolute: ${COURSE_WS}"
}

preflight() {
  CURRENT_STEP="environment preflight"

  [[ "$(uname -s)" == "Linux" ]] || die "This installer requires Linux on RISC-V"
  [[ -r /etc/os-release ]] || die "Cannot read /etc/os-release"

  # shellcheck disable=SC1091
  source /etc/os-release
  [[ "${ID:-}" == "${TARGET_BIANBU_OS_ID}" ]] || \
    die "Bianbu ${TARGET_BIANBU_MIN_VERSION} or newer is required; detected ${ID:-unknown}"
  command -v dpkg >/dev/null 2>&1 || die "dpkg is required on Bianbu"
  dpkg --compare-versions "${VERSION_ID:-0}" ge "${TARGET_BIANBU_MIN_VERSION}" || \
    die "Bianbu ${TARGET_BIANBU_MIN_VERSION} or newer is required; detected ${VERSION_ID:-unknown}"
  PLATFORM_ID="Bianbu"
  PLATFORM_VERSION="${VERSION_ID}"
  [[ -x /usr/bin/apt-get ]] || die "/usr/bin/apt-get is required on Bianbu"

  local model
  [[ -r /proc/device-tree/model ]] || die "Cannot read K3 board model"
  model="$(tr -d '\0' </proc/device-tree/model)"
  [[ "${model}" =~ K3[[:space:]-]+Pico[[:space:]-]+ITX ]] || \
    die "K3 Pico-ITX is required; detected ${model:-unknown}"

  local arch
  arch="$(uname -m)"
  if [[ "${arch}" != "riscv64" ]]; then
    die "riscv64 is required; detected ${arch}"
  fi

  [[ "${EUID}" -ne 0 ]] || die "Run this script as a normal user; sudo is invoked only when needed"
  [[ -d "${COURSE_SRC}" ]] || die "Course src directory not found: ${COURSE_SRC}"

  if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "${TARGET_ROS_DISTRO}" ]]; then
    die "Another ROS distribution is active (${ROS_DISTRO}); start a clean shell"
  fi

  local available_kb
  available_kb="$(df -Pk "${HOME}" | awk 'NR == 2 {print $4}')"
  if ((available_kb < 15728640)); then
    log_warn "Less than 15 GiB is available under ${HOME}"
  fi

  log_ok "Platform: ${PLATFORM_ID} ${PLATFORM_VERSION} (${arch}), target ROS 2 ${TARGET_ROS_DISTRO}"
  log_info "Course root: ${COURSE_ROOT}"
  log_info "Managed workspace: ${COURSE_WS}"
}

acquire_lock() {
  [[ "${DRY_RUN}" == true || "${VERIFY_ONLY}" == true ]] && return 0
  command -v flock >/dev/null 2>&1 || die "flock is required (package: util-linux)"
  local lock_file="${XDG_RUNTIME_DIR:-/tmp}/ros2-course-k3-setup-${UID}.lock"
  exec 9>"${lock_file}"
  flock -n "${LOCK_FD}" || die "Another setup_course_k3.sh process is running"
}

apt_install() {
  (($# > 0)) || return 0
  run sudo -n /usr/bin/apt-get install -y "$@"
}

apt_install_best_effort() {
  (($# > 0)) || return 0
  run_best_effort sudo -n /usr/bin/apt-get install -y "$@"
}

install_base_tools() {
  CURRENT_STEP="base system dependencies"
  log_info "Installing base build and Python dependencies"
  run sudo -n /usr/bin/apt-get update
  apt_install "${BASE_APT_PACKAGES[@]}"
  apt_install_best_effort "${BEST_EFFORT_APT_PACKAGES[@]}"

  if [[ "${DRY_RUN}" == false ]]; then
    if ! command -v colcon >/dev/null 2>&1; then
      log_warn "python3-colcon-common-extensions is unavailable; installing colcon via pip"
      run python3 -m pip install --user colcon-common-extensions
      export PATH="${HOME}/.local/bin:${PATH}"
    fi
    if ! command -v rosdep >/dev/null 2>&1; then
      log_warn "python3-rosdep is unavailable; installing rosdep via pip"
      run python3 -m pip install --user rosdep
      export PATH="${HOME}/.local/bin:${PATH}"
    fi
  fi
}

prepare_ros_repository() {
  if [[ "${DRY_RUN}" == true ]]; then
    log_info "Would use the configured Bianbu apt repositories for ROS 2 Humble"
    return 0
  fi
  apt-cache show ros-humble-ros-base >/dev/null 2>&1 || \
    die "ros-humble-ros-base is unavailable from the configured Bianbu apt repositories"
  log_ok "Bianbu ROS 2 Humble packages are available"
}

install_ros() {
  CURRENT_STEP="ROS 2 Humble installation"
  prepare_ros_repository
  apt_install "${REQUIRED_ROS_PACKAGES[@]}"
  apt_install_best_effort "${BEST_EFFORT_ROS_PACKAGES[@]}"

  if [[ "${DRY_RUN}" == false ]]; then
    [[ -f "/opt/ros/${TARGET_ROS_DISTRO}/setup.bash" ]] || \
      die "ROS installation completed without /opt/ros/${TARGET_ROS_DISTRO}/setup.bash"
  fi
}

source_ros() {
  local setup_file="/opt/ros/${TARGET_ROS_DISTRO}/setup.bash"
  if [[ ! -f "${setup_file}" ]]; then
    [[ "${DRY_RUN}" == true ]] && return 0
    die "ROS setup file not found: ${setup_file}"
  fi

  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
  [[ "${ROS_DISTRO:-}" == "${TARGET_ROS_DISTRO}" ]] || die "Failed to activate ROS 2 Humble"
}

initialize_rosdep() {
  CURRENT_STEP="rosdep initialization"
  command -v rosdep >/dev/null 2>&1 || {
    log_warn "rosdep is unavailable; skipping rosdep initialization"
    return 0
  }
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    log_warn "rosdep system sources are not initialized; using the reviewed apt package list only"
    return 0
  fi
  if [[ "${DRY_RUN}" == true ]]; then
    print_command rosdep update --rosdistro "${TARGET_ROS_DISTRO}"
    return 0
  fi
  if rosdep update --rosdistro "${TARGET_ROS_DISTRO}"; then
    ROSDEP_READY=true
  else
    log_warn "rosdep update failed; using the reviewed apt package list only"
  fi
}

discover_package_names() {
  colcon list --base-paths "$@" | awk '{print $1}' | LC_ALL=C sort
}

course_package_base_paths() {
  find "${COURSE_SRC}" -mindepth 1 -maxdepth 1 -type d -print | LC_ALL=C sort
}

discover_source_package_names() {
  local course_base_paths=()
  mapfile -t course_base_paths < <(course_package_base_paths)
  discover_package_names "${course_base_paths[@]}"
}

discover_installed_package_names() {
  [[ -d "${COURSE_WS}/install" ]] || return 0
  find -L "${COURSE_WS}/install" \
    -path '*/share/ament_index/resource_index/packages/*' \
    -type f -printf '%f\n' | LC_ALL=C sort -u
}

verify_installed_packages() {
  local source_packages=()
  local installed_packages=()
  local missing_packages unexpected_packages
  local package_name metadata_missing=0

  mapfile -t source_packages < <(discover_source_package_names)
  mapfile -t installed_packages < <(discover_installed_package_names)

  if [[ "${source_packages[*]}" != "${installed_packages[*]}" ]]; then
    missing_packages="$(comm -23 \
      <(printf '%s\n' "${source_packages[@]}") \
      <(printf '%s\n' "${installed_packages[@]}"))"
    unexpected_packages="$(comm -13 \
      <(printf '%s\n' "${source_packages[@]}") \
      <(printf '%s\n' "${installed_packages[@]}"))"
    [[ -z "${missing_packages}" ]] || log_warn "Packages missing from install: ${missing_packages}"
    [[ -z "${unexpected_packages}" ]] || log_warn "Unexpected installed packages: ${unexpected_packages}"
    return 1
  fi

  for package_name in "${source_packages[@]}"; do
    if ! find -L "${COURSE_WS}/install" \
      -path "*/share/${package_name}/package.xml" \
      -type f -print -quit | grep -q .; then
      log_warn "Installed package metadata is missing: ${package_name}/package.xml"
      ((metadata_missing += 1))
    fi
  done

  ((metadata_missing == 0)) || return 1
  log_ok "All ${#source_packages[@]} source packages have install resources and package metadata"
}

assert_unique_packages() {
  local duplicate_names
  local course_base_paths=()
  mapfile -t course_base_paths < <(course_package_base_paths)
  duplicate_names="$(colcon list --base-paths "${course_base_paths[@]}" | \
    awk '{print $1}' | LC_ALL=C sort | uniq -d)"
  [[ -z "${duplicate_names}" ]] || die "Duplicate ROS package names detected: ${duplicate_names}"
}

sync_workspace() {
  CURRENT_STEP="managed workspace synchronization"
  local marker="${COURSE_WS}/.ros2-course-k3-managed"
  local managed_path
  local excludes=(
    --exclude=.git/
    --exclude=.venv/
    --exclude=__pycache__/
    --exclude='*.pyc'
    --exclude=build/
    --exclude=install/
    --exclude=log/
  )

  if [[ "${DRY_RUN}" == true ]]; then
    print_command mkdir -p "${COURSE_WS}/src/course"
    print_command rsync -a --delete "${excludes[@]}" "${COURSE_SRC}/" "${COURSE_WS}/src/course/"
    return 0
  fi

  assert_unique_packages

  for managed_path in \
    "${COURSE_WS}" \
    "${COURSE_WS}/src" \
    "${COURSE_WS}/src/course"; do
    [[ ! -L "${managed_path}" ]] || \
      die "Refusing to use a symbolic link in the managed workspace: ${managed_path}"
  done

  if [[ -d "${COURSE_WS}" && ! -f "${marker}" ]]; then
    if [[ -n "$(find "${COURSE_WS}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
      die "Refusing to modify an unmanaged non-empty workspace: ${COURSE_WS}"
    fi
  fi

  mkdir -p "${COURSE_WS}/src/course"
  managed_path="${COURSE_WS}/src/course"
  [[ -d "${managed_path}" && ! -L "${managed_path}" ]] || \
    die "Managed workspace target is not a regular directory: ${managed_path}"
  touch "${marker}"

  rsync -a --delete "${excludes[@]}" "${COURSE_SRC}/" "${COURSE_WS}/src/course/"

  mapfile -t source_packages < <(discover_source_package_names)
  mapfile -t workspace_packages < <(
    discover_package_names "${COURSE_WS}/src/course"
  )
  if [[ "${source_packages[*]}" != "${workspace_packages[*]}" ]]; then
    die "Workspace package discovery differs from the course source"
  fi
  log_ok "Synchronized ${#workspace_packages[@]} ROS packages"
}

install_workspace_dependencies() {
  CURRENT_STEP="course and lab dependencies"
  log_info "Installing reviewed dependencies for package and script-based labs"

  if [[ "${ROSDEP_READY}" == true && "${DRY_RUN}" == false ]]; then
    log_info "Checking the reviewed apt package list against rosdep metadata"
    run_best_effort rosdep check \
      --from-paths "${COURSE_WS}/src/course" \
      --ignore-src \
      --rosdistro "${TARGET_ROS_DISTRO}" \
      --skip-keys ament_python
  else
    log_warn "rosdep sources are not ready; relying on the reviewed ROS package list only"
  fi
}

install_hardware_profile() {
  [[ "${WITH_HARDWARE}" == true ]] || return 0
  CURRENT_STEP="hardware profile"
  log_info "Installing camera and serial dependencies (best effort on riscv64)"
  apt_install_best_effort "${HARDWARE_ROS_PACKAGES[@]}"
}

install_ml_profile() {
  [[ "${WITH_ML}" == true ]] || return 0
  CURRENT_STEP="ML profile"
  log_info "Installing ML packages into ${ML_VENV} (riscv64 wheels may compile from source)"
  run mkdir -p "$(dirname "${ML_VENV}")"
  if [[ ! -x "${ML_VENV}/bin/python" ]]; then
    run python3 -m venv --system-site-packages "${ML_VENV}"
  fi
  run "${ML_VENV}/bin/python" -m pip install "${ML_PIP_PACKAGES[@]}"
}

build_workspace() {
  CURRENT_STEP="course workspace build"
  if [[ "${DRY_RUN}" == true ]]; then
    print_command colcon build \
      --base-paths "${COURSE_WS}/src/course" \
      --symlink-install \
      --cmake-clean-cache \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    return 0
  fi

  (
    cd "${COURSE_WS}"
    colcon build \
      --base-paths "${COURSE_WS}/src/course" \
      --symlink-install \
      --cmake-clean-cache \
      --event-handlers console_cohesion+ \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
  )
  [[ -f "${COURSE_WS}/install/setup.bash" ]] || die "Course build did not produce install/setup.bash"

  set +u
  # shellcheck disable=SC1091
  source "${COURSE_WS}/install/setup.bash"
  set -u
  log_ok "Course workspace built successfully"
}

test_workspace() {
  [[ "${RUN_TESTS}" == true ]] || return 0
  CURRENT_STEP="course workspace tests"
  if [[ "${DRY_RUN}" == true ]]; then
    print_command colcon test \
      --base-paths "${COURSE_WS}/src/course" \
      --executor sequential
    print_command colcon test-result --test-result-base "${COURSE_WS}/build" --verbose
    return 0
  fi

  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((20 + $$ % 80))}"
  (
    cd "${COURSE_WS}"
    colcon test \
      --base-paths "${COURSE_WS}/src/course" \
      --executor sequential \
      --event-handlers console_cohesion+ \
      --return-code-on-test-failure
    colcon test-result --test-result-base "${COURSE_WS}/build" --verbose
  )
}

write_environment_file() {
  CURRENT_STEP="shell environment configuration"
  if [[ "${DRY_RUN}" == true ]]; then
    log_info "Would generate ${ENV_FILE} and update the managed ~/.bashrc block"
    return 0
  fi

  mkdir -p "${ENV_DIR}"
  local env_tmp bashrc_tmp bashrc_file
  env_tmp="$(mktemp)"
  bashrc_file="${HOME}/.bashrc"

  {
    printf '# Generated by %q. Manual edits will be replaced.\n' "${COURSE_ROOT}/setup_course_k3.sh"
    printf 'if [[ -f %q ]]; then\n' "/opt/ros/${TARGET_ROS_DISTRO}/setup.bash"
    printf '  source %q\n' "/opt/ros/${TARGET_ROS_DISTRO}/setup.bash"
    printf 'fi\n'
    printf 'if [[ -f %q ]]; then\n' "${COURSE_WS}/install/setup.bash"
    printf '  source %q\n' "${COURSE_WS}/install/setup.bash"
    printf 'fi\n'
    printf 'export ROS2_K3_COURSE_ROOT=%q\n' "${COURSE_ROOT}"
    printf 'export ROS2_K3_COURSE_WS=%q\n' "${COURSE_WS}"
    printf 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n'
    printf 'export RCUTILS_COLORIZED_OUTPUT=1\n'
    printf 'export RCUTILS_LOGGING_USE_STDOUT=1\n'
    printf '# Set ROS_DOMAIN_ID to the value confirmed for the x86 Gazebo host before cross-machine validation\n'
    # shellcheck disable=SC2016
    printf 'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"\n'
    if [[ -x "${ML_VENV}/bin/python" ]]; then
      printf 'export ROS2_K3_COURSE_ML_PYTHON=%q\n' "${ML_VENV}/bin/python"
    fi
    printf "export PATH=\"\${HOME}/.local/bin:\${PATH}\"\n"
    printf "alias k3cw='cd \"\${ROS2_K3_COURSE_WS}\"'\n"
    printf "alias k3cs='source \"\${ROS2_K3_COURSE_WS}/install/setup.bash\"'\n"
    printf "alias k3cb='cd \"\${ROS2_K3_COURSE_WS}\" && colcon build --base-paths src/course --symlink-install'\n"
  } > "${env_tmp}"
  install -m 0644 "${env_tmp}" "${ENV_FILE}"
  rm -f "${env_tmp}"

  [[ ! -L "${bashrc_file}" ]] || \
    die "Refusing to replace symbolic link ${bashrc_file}"
  touch "${bashrc_file}"
  bashrc_tmp="$(mktemp "${bashrc_file}.tmp.XXXXXX")"
  if ! awk -v begin="${BASHRC_BEGIN}" -v end="${BASHRC_END}" '
    $0 == begin {
      if (seen_begin || seen_end) exit 2
      seen_begin = 1
      skip = 1
      next
    }
    $0 == end {
      if (!seen_begin || seen_end) exit 2
      seen_end = 1
      skip = 0
      next
    }
    !skip {print}
    END {if (seen_begin != seen_end) exit 2}
  ' "${bashrc_file}" > "${bashrc_tmp}"; then
    rm -f "${bashrc_tmp}"
    die "Refusing to replace a malformed managed block in ${bashrc_file}"
  fi
  {
    printf '\n%s\n' "${BASHRC_BEGIN}"
    printf 'alias k3env=%q\n' "source ${ENV_FILE}"
    printf '%s\n' "${BASHRC_END}"
  } >> "${bashrc_tmp}"
  chmod --reference="${bashrc_file}" "${bashrc_tmp}"
  mv "${bashrc_tmp}" "${bashrc_file}"
  log_ok "Managed shell environment written to ${ENV_FILE}"
}

check() {
  local description=$1
  shift
  if "$@"; then
    log_ok "${description}"
    return 0
  fi
  log_warn "${description}"
  return 1
}

verify_installation() {
  CURRENT_STEP="installation verification"
  [[ "${DRY_RUN}" == true ]] && {
    log_info "Dry-run complete; verification requires installed artifacts"
    return 0
  }

  local failures=0
  source_ros
  if [[ -f "${COURSE_WS}/install/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1091
    source "${COURSE_WS}/install/setup.bash"
    set -u
  fi

  check "ROS 2 Humble setup exists" test -f "/opt/ros/${TARGET_ROS_DISTRO}/setup.bash" || ((failures += 1))
  check "Bianbu ROS 2 Humble packages are available" \
    apt-cache show ros-humble-ros-base >/dev/null 2>&1 || ((failures += 1))
  check "Course workspace overlay exists" test -f "${COURSE_WS}/install/setup.bash" || ((failures += 1))
  check "Managed shell environment exists" test -f "${ENV_FILE}" || ((failures += 1))
  check "CycloneDDS RMW is installed" ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null || ((failures += 1))
  check "C++ demo nodes are installed" ros2 pkg prefix demo_nodes_cpp >/dev/null || ((failures += 1))
  check "TF2 tools are installed" ros2 pkg prefix tf2_tools >/dev/null || ((failures += 1))
  check "TurtleSim is installed" ros2 pkg prefix turtlesim >/dev/null || ((failures += 1))

  if [[ -f "${COURSE_WS}/install/setup.bash" ]]; then
    check "Representative course package is discoverable" \
      ros2 pkg prefix lifecycle_demo_cpp >/dev/null || ((failures += 1))
    mapfile -t source_packages < <(discover_source_package_names)
    mapfile -t workspace_packages < <(
      discover_package_names "${COURSE_WS}/src/course"
    )
    if [[ "${source_packages[*]}" == "${workspace_packages[*]}" ]]; then
      log_ok "All ${#workspace_packages[@]} source packages are present in the workspace"
    else
      log_warn "Workspace package list differs from the source tree"
      ((failures += 1))
    fi
    if ! verify_installed_packages; then
      ((failures += 1))
    fi
  fi

  if [[ "${WITH_ML}" == true ]]; then
    check "ML profile imports" "${ML_VENV}/bin/python" -c \
      'import filterpy, openai' || ((failures += 1))
  fi

  check "Installer shell syntax" bash -n "${COURSE_ROOT}/setup_course_k3.sh" || ((failures += 1))
  if command -v shellcheck >/dev/null 2>&1; then
    check "Installer ShellCheck" shellcheck "${COURSE_ROOT}/setup_course_k3.sh" || ((failures += 1))
  fi

  ((failures == 0)) || die "Verification failed with ${failures} error(s)"
  log_ok "Installation verification passed"
}

main() {
  parse_args "$@"
  preflight
  acquire_lock

  if [[ "${VERIFY_ONLY}" == true ]]; then
    verify_installation
    return 0
  fi

  if [[ "${REFRESH_ENV}" == true ]]; then
    source_ros
    write_environment_file
    verify_installation
    log_ok "Shell environment refreshed"
    return 0
  fi

  if [[ "${DRY_RUN}" == false ]]; then
    CURRENT_STEP="non-interactive sudo preflight"
    sudo -n /usr/bin/apt-get --version >/dev/null 2>&1 || \
      die "Non-interactive sudo permission for apt-get is required"
  fi

  install_base_tools
  install_ros
  source_ros
  initialize_rosdep

  sync_workspace
  install_workspace_dependencies
  install_hardware_profile
  install_ml_profile
  build_workspace
  test_workspace

  write_environment_file
  verify_installation

  log_ok "Setup completed"
  log_info "Activate the K3 environment: source ${ENV_FILE}"
}

main "$@"
