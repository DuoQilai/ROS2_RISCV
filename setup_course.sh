#!/usr/bin/env bash
# RISC-V ROS2 机器人操作系统编程技术 - 课程环境安装器（板卡端）
# Target: openEuler 24.03 LTS (RISC-V) + ROS 2 Humble (openEuler ROS SIG repository)
# Gazebo and RViz2 have no riscv64 packages; they run on the Windows x86 host
# and join the same LAN DDS domain as the board.
# Reference: https://docs.openeuler.org/zh/docs/24.03_LTS_SP3/tools/application/ros/installation_and_deployment.html

set -Eeuo pipefail
IFS=$'\n\t'

TARGET_OS_ID="openEuler"
TARGET_OS_VERSION="24.03"
TARGET_ROS_DISTRO="humble"
ROS_REPO_FILE="/etc/yum.repos.d/ROS.repo"
ROS_REPO_URL_RISCV="https://build-repo.tarsier-infra.isrc.ac.cn/openEuler:/ROS/24.03/"

COURSE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COURSE_WS="${ROS2_COURSE_WS:-${HOME}/ros2_course_ws}"
COURSE_SRC="${COURSE_ROOT}/src"
LAB_SRC="${COURSE_ROOT}/src/lab_code"
ML_VENV="${ROS2_COURSE_ML_VENV:-${HOME}/.venvs/ros2-course-ml}"
ENV_DIR="${XDG_CONFIG_HOME:-${HOME}/.config}/ros2-course"
ENV_FILE="${ENV_DIR}/env.bash"

WITH_ML=false
WITH_HARDWARE=false
RUN_TESTS=false
VERIFY_ONLY=false
DRY_RUN=false
REFRESH_ENV=false

CURRENT_STEP="startup"
LOCK_FD=9

readonly BASHRC_BEGIN="# >>> ROS2 course environment >>>"
readonly BASHRC_END="# <<< ROS2 course environment <<<"

BASE_DNF_PACKAGES=(
  ca-certificates
  cmake
  curl
  gcc
  gcc-c++
  git
  make
  pkgconf-pkg-config
  procps-ng
  python3-devel
  python3-pip
  python3-setuptools
  python3-wheel
  rsync
  tar
  tmux
  unzip
  which
  xz
)

BEST_EFFORT_DNF_PACKAGES=(
  python3-colcon-common-extensions
  python3-rosdep
  python3-vcstool
  python3-matplotlib
  python3-numpy
  python3-opencv
  python3-pytest
  python3-pytest-cov
  python3-pyserial
  python3-pyyaml
  python3-requests
  python3-scikit-learn
  python3-scipy
)

REQUIRED_ROS_PACKAGES=(
  ros-humble-ros-base
  ros-humble-rmw-cyclonedds-cpp
  ros-humble-turtlesim
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
  ros-humble-teleop-twist-keyboard
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
Usage: bash setup_course.sh [options]

Target platform: openEuler 24.03 LTS on RISC-V (riscv64), ROS 2 Humble from
the openEuler ROS SIG repository. This is the board-side installer; Gazebo,
RViz2 run on the Windows x86 host and share one LAN DDS domain.

Default action:
  Configure the ROS SIG dnf repository, install ROS 2 Humble and base
  dependencies, synchronize the course into ~/ros2_course_ws, build all ROS
  packages, configure ~/.bashrc, and verify.

Options:
  --with-ml             Install ML dependencies in an isolated venv
                        (wheels on riscv64 may need to compile)
  --with-hardware       Install camera / serial / fiducial dependencies
  --workspace PATH      Use a managed workspace other than ~/ros2_course_ws
  --run-tests           Run colcon tests after a successful build
  --verify              Verify an existing installation without changing it
  --refresh-env         Regenerate the shell environment without reinstalling
  --dry-run             Print mutating commands without executing them
  --help                Show this help

Environment overrides:
  ROS2_COURSE_WS, ROS2_COURSE_ML_VENV
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

  [[ "$(uname -s)" == "Linux" ]] || die "This installer requires openEuler Linux on RISC-V"
  [[ -r /etc/os-release ]] || die "Cannot read /etc/os-release"

  # shellcheck disable=SC1091
  source /etc/os-release
  [[ "${ID:-}" == "${TARGET_OS_ID}" ]] || \
    die "Unsupported operating system: ${ID:-unknown}; this installer only supports openEuler ${TARGET_OS_VERSION} (Ubuntu hosts are no longer the course platform)"
  [[ "${VERSION_ID:-}" == "${TARGET_OS_VERSION}"* ]] || \
    die "openEuler ${TARGET_OS_VERSION} is required; detected ${VERSION_ID:-unknown}"

  local arch
  arch="$(uname -m)"
  if [[ "${arch}" != "riscv64" && "${DRY_RUN}" == false ]]; then
    die "riscv64 is required; detected ${arch}. The openEuler ROS SIG RISC-V repository only serves riscv64; x86_64/aarch64 boards must replace ROS_REPO_URL_RISCV with the EulerMaker repository"
  fi

  [[ "${EUID}" -ne 0 ]] || die "Run this script as a normal user; sudo is invoked only when needed"
  [[ -d "${COURSE_SRC}" ]] || die "Course src directory not found: ${COURSE_SRC}"
  [[ -d "${LAB_SRC}" ]] || die "Course lab_code directory not found: ${LAB_SRC}"

  if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "${TARGET_ROS_DISTRO}" ]]; then
    die "Another ROS distribution is active (${ROS_DISTRO}); start a clean shell"
  fi

  local available_kb
  available_kb="$(df -Pk "${HOME}" | awk 'NR == 2 {print $4}')"
  if ((available_kb < 15728640)); then
    log_warn "Less than 15 GiB is available under ${HOME}"
  fi

  log_ok "Platform: openEuler ${VERSION_ID} (${arch}), target ROS 2 ${TARGET_ROS_DISTRO}"
  log_info "Course root: ${COURSE_ROOT}"
  log_info "Managed workspace: ${COURSE_WS}"
}

acquire_lock() {
  [[ "${DRY_RUN}" == true || "${VERIFY_ONLY}" == true ]] && return 0
  command -v flock >/dev/null 2>&1 || die "flock is required (package: util-linux)"
  local lock_file="${XDG_RUNTIME_DIR:-/tmp}/ros2-course-setup-${UID}.lock"
  exec 9>"${lock_file}"
  flock -n "${LOCK_FD}" || die "Another setup_course.sh process is running"
}

dnf_install() {
  (($# > 0)) || return 0
  run sudo dnf install -y "$@"
}

dnf_install_best_effort() {
  (($# > 0)) || return 0
  run_best_effort sudo dnf install -y --skip-broken --allowerasing "$@"
}

install_base_tools() {
  CURRENT_STEP="base system dependencies"
  log_info "Installing base build and Python dependencies"
  run sudo dnf makecache || log_warn "dnf makecache failed; continuing with cached metadata"
  dnf_install "${BASE_DNF_PACKAGES[@]}"
  dnf_install_best_effort "${BEST_EFFORT_DNF_PACKAGES[@]}"

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

configure_ros_repository() {
  if [[ -s "${ROS_REPO_FILE}" ]] && grep -q "openEuler:/ROS\|ROS-SIG" "${ROS_REPO_FILE}"; then
    log_ok "openEuler ROS SIG dnf repository is already configured"
    return 0
  fi

  if [[ "${DRY_RUN}" == true ]]; then
    log_info "Would configure ${ROS_REPO_FILE} for the openEuler ROS SIG Humble repository (riscv64)"
    return 0
  fi

  sudo tee "${ROS_REPO_FILE}" >/dev/null <<EOF
[openEulerROS-humble]
name=openEulerROS-humble
baseurl=${ROS_REPO_URL_RISCV}
enabled=1
gpgcheck=0
EOF
  log_ok "Configured ${ROS_REPO_FILE}"
}

install_ros() {
  CURRENT_STEP="ROS 2 Humble installation"
  configure_ros_repository
  run sudo dnf makecache
  dnf_install "${REQUIRED_ROS_PACKAGES[@]}"
  dnf_install_best_effort "${BEST_EFFORT_ROS_PACKAGES[@]}"

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
    run_best_effort sudo rosdep init
  fi
  run_best_effort rosdep update --rosdistro "${TARGET_ROS_DISTRO}"
}

discover_package_names() {
  colcon list --base-paths "$@" | awk '{print $1}' | LC_ALL=C sort
}

course_package_base_paths() {
  find "${COURSE_SRC}" -mindepth 1 -maxdepth 1 -type d \
    ! -path "${LAB_SRC}" -print | LC_ALL=C sort
}

discover_source_package_names() {
  local course_base_paths=()
  mapfile -t course_base_paths < <(course_package_base_paths)
  discover_package_names "${course_base_paths[@]}" "${LAB_SRC}"
}

assert_unique_packages() {
  local duplicate_names
  local course_base_paths=()
  mapfile -t course_base_paths < <(course_package_base_paths)
  duplicate_names="$(colcon list --base-paths "${course_base_paths[@]}" "${LAB_SRC}" | \
    awk '{print $1}' | LC_ALL=C sort | uniq -d)"
  [[ -z "${duplicate_names}" ]] || die "Duplicate ROS package names detected: ${duplicate_names}"
}

sync_workspace() {
  CURRENT_STEP="managed workspace synchronization"
  local marker="${COURSE_WS}/.ros2-course-managed"
  local excludes=(
    --exclude=.git/
    --exclude=.venv/
    --exclude=__pycache__/
    --exclude='*.pyc'
    --exclude=build/
    --exclude=install/
    --exclude=log/
    --exclude=lab_code/
  )

  if [[ "${DRY_RUN}" == true ]]; then
    print_command mkdir -p "${COURSE_WS}/src/course" "${COURSE_WS}/src/labs"
    print_command rsync -a --delete "${excludes[@]}" "${COURSE_SRC}/" "${COURSE_WS}/src/course/"
    print_command rsync -a --delete "${excludes[@]}" "${LAB_SRC}/" "${COURSE_WS}/src/labs/"
    return 0
  fi

  assert_unique_packages

  if [[ -d "${COURSE_WS}" && ! -f "${marker}" ]]; then
    if [[ -n "$(find "${COURSE_WS}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
      die "Refusing to modify an unmanaged non-empty workspace: ${COURSE_WS}"
    fi
  fi

  mkdir -p "${COURSE_WS}/src/course" "${COURSE_WS}/src/labs"
  touch "${marker}"

  rsync -a --delete "${excludes[@]}" "${COURSE_SRC}/" "${COURSE_WS}/src/course/"
  rsync -a --delete "${excludes[@]}" "${LAB_SRC}/" "${COURSE_WS}/src/labs/"

  mapfile -t source_packages < <(discover_source_package_names)
  mapfile -t workspace_packages < <(discover_package_names "${COURSE_WS}/src")
  if [[ "${source_packages[*]}" != "${workspace_packages[*]}" ]]; then
    die "Workspace package discovery differs from the course source"
  fi
  log_ok "Synchronized ${#workspace_packages[@]} ROS packages"
}

install_workspace_dependencies() {
  CURRENT_STEP="course and lab dependencies"
  log_info "Installing reviewed dependencies for package and script-based labs"

  local rosdep_keys=()
  local pkg
  if command -v rosdep >/dev/null 2>&1 && [[ "${DRY_RUN}" == false ]]; then
    set +e
    mapfile -t rosdep_keys < <(
      rosdep keys --from-paths "${COURSE_WS}/src" --ignore-src 2>/dev/null | \
        grep '^ros-humble-' | sort -u
    )
    set -e
    if ((${#rosdep_keys[@]} > 0)); then
      log_info "Resolving ${#rosdep_keys[@]} ros-humble-* keys through the openEuler ROS SIG repository"
      for pkg in "${rosdep_keys[@]}"; do
        run_best_effort sudo dnf install -y --skip-broken "${pkg}"
      done
    fi
    run_best_effort rosdep install \
      --from-paths "${COURSE_WS}/src" \
      --ignore-src \
      --rosdistro "${TARGET_ROS_DISTRO}" \
      --skip-keys ament_python \
      -r -y
    run_best_effort rosdep check \
      --from-paths "${COURSE_WS}/src" \
      --ignore-src \
      --rosdistro "${TARGET_ROS_DISTRO}" \
      --skip-keys ament_python
  else
    log_warn "rosdep is unavailable; relying on the best-effort ROS package list only"
  fi
}

install_hardware_profile() {
  [[ "${WITH_HARDWARE}" == true ]] || return 0
  CURRENT_STEP="hardware profile"
  log_info "Installing camera and serial dependencies (best effort on riscv64)"
  dnf_install_best_effort "${HARDWARE_ROS_PACKAGES[@]}"
  log_warn "realsense2_camera and aruco packages have no openEuler riscv64 builds; build them from source when the hardware is attached"
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
      --base-paths "${COURSE_WS}/src" \
      --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    return 0
  fi

  (
    cd "${COURSE_WS}"
    colcon build \
      --base-paths "${COURSE_WS}/src" \
      --symlink-install \
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
    print_command colcon test --base-paths "${COURSE_WS}/src" --executor sequential
    print_command colcon test-result --test-result-base "${COURSE_WS}/build" --verbose
    return 0
  fi

  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((20 + $$ % 200))}"
  (
    cd "${COURSE_WS}"
    colcon test \
      --base-paths "${COURSE_WS}/src" \
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
  bashrc_tmp="$(mktemp)"
  bashrc_file="${HOME}/.bashrc"

  {
    printf '# Generated by %q. Manual edits will be replaced.\n' "${COURSE_ROOT}/setup_course.sh"
    printf 'if [[ -f %q ]]; then\n' "/opt/ros/${TARGET_ROS_DISTRO}/setup.bash"
    printf '  source %q\n' "/opt/ros/${TARGET_ROS_DISTRO}/setup.bash"
    printf 'fi\n'
    printf 'if [[ -f %q ]]; then\n' "${COURSE_WS}/install/setup.bash"
    printf '  source %q\n' "${COURSE_WS}/install/setup.bash"
    printf 'fi\n'
    printf 'export ROS2_COURSE_ROOT=%q\n' "${COURSE_ROOT}"
    printf 'export ROS2_COURSE_WS=%q\n' "${COURSE_WS}"
    printf 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n'
    printf 'export RCUTILS_COLORIZED_OUTPUT=1\n'
    printf 'export RCUTILS_LOGGING_USE_STDOUT=1\n'
    printf '# Keep ROS_DOMAIN_ID identical to the Windows x86 host so its RViz2 can see board nodes\n'
    printf 'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"\n'
    if [[ -x "${ML_VENV}/bin/python" ]]; then
      printf 'export ROS2_COURSE_ML_PYTHON=%q\n' "${ML_VENV}/bin/python"
    fi
    printf "export PATH=\"\${HOME}/.local/bin:\${PATH}\"\n"
    printf "alias cw='cd \"\${ROS2_COURSE_WS}\"'\n"
    printf "alias cs='source \"\${ROS2_COURSE_WS}/install/setup.bash\"'\n"
    printf "alias cb='cd \"\${ROS2_COURSE_WS}\" && colcon build --symlink-install'\n"
  } > "${env_tmp}"
  install -m 0644 "${env_tmp}" "${ENV_FILE}"
  rm -f "${env_tmp}"

  touch "${bashrc_file}"
  awk -v begin="${BASHRC_BEGIN}" -v end="${BASHRC_END}" '
    $0 == begin {skip = 1; next}
    $0 == end {skip = 0; next}
    !skip {print}
  ' "${bashrc_file}" > "${bashrc_tmp}"
  {
    cat "${bashrc_tmp}"
    printf '\n%s\n' "${BASHRC_BEGIN}"
    printf 'source %q\n' "${ENV_FILE}"
    printf '%s\n' "${BASHRC_END}"
  } > "${bashrc_file}"
  rm -f "${bashrc_tmp}"
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
  check "openEuler ROS SIG repository is configured" test -s "${ROS_REPO_FILE}" || ((failures += 1))
  check "Course workspace overlay exists" test -f "${COURSE_WS}/install/setup.bash" || ((failures += 1))
  check "Managed shell environment exists" test -f "${ENV_FILE}" || ((failures += 1))
  check "CycloneDDS RMW is installed" ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null || ((failures += 1))
  check "TurtleSim is installed" ros2 pkg prefix turtlesim >/dev/null || ((failures += 1))

  if [[ -f "${COURSE_WS}/install/setup.bash" ]]; then
    check "Representative course package is discoverable" \
      ros2 pkg prefix course_lab_utils >/dev/null || ((failures += 1))
    mapfile -t source_packages < <(discover_source_package_names)
    mapfile -t workspace_packages < <(discover_package_names "${COURSE_WS}/src")
    if [[ "${source_packages[*]}" == "${workspace_packages[*]}" ]]; then
      log_ok "All ${#workspace_packages[@]} source packages are present in the workspace"
    else
      log_warn "Workspace package list differs from the source tree"
      ((failures += 1))
    fi
  fi

  if [[ "${WITH_ML}" == true ]]; then
    check "ML profile imports" "${ML_VENV}/bin/python" -c \
      'import filterpy, openai' || ((failures += 1))
  fi

  check "Installer shell syntax" bash -n "${COURSE_ROOT}/setup_course.sh" || ((failures += 1))
  if command -v shellcheck >/dev/null 2>&1; then
    check "Installer ShellCheck" shellcheck "${COURSE_ROOT}/setup_course.sh" || ((failures += 1))
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
    sudo -v
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
  log_info "Open a new terminal or run: source ${ENV_FILE}"
}

main "$@"
