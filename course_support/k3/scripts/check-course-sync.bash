#!/usr/bin/env bash

set -Eeuo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
course_repo=${ROS2_RISCV_LOCAL_REPO:-$(cd -- "$script_dir/../../.." && pwd)}

sha256_file() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  else
    shasum -a 256 "$1" | awk '{print $1}'
  fi
}

sha256_stream() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum | awk '{print $1}'
  else
    shasum -a 256 | awk '{print $1}'
  fi
}

source_hash() {
  local repo=$1
  git -C "$repo" ls-files --cached --others --exclude-standard -- setup_course_k3.sh src_k3_pico_itx |
    LC_ALL=C sort |
    while IFS= read -r path; do
      case "$path" in
        *.pyc|*/.venv/*|*/__pycache__/*|*/build/*|*/install/*|*/log/*) continue ;;
      esac
      [[ -f "$repo/$path" ]] || continue
      printf '%s %s\n' "$path" "$(sha256_file "$repo/$path")"
    done |
    sha256_stream
}

local_head=$(git -C "$course_repo" rev-parse HEAD)
local_source_hash=$(source_hash "$course_repo")
printf 'LOCAL_HEAD=%s\n' "$local_head"
printf 'LOCAL_SOURCE_SHA256=%s\n' "$local_source_hash"
git -C "$course_repo" status --short
printf 'LOCAL_STATUS_END\n'

set +e
remote_state=$(ssh -o BatchMode=yes -o ConnectTimeout=10 -o StrictHostKeyChecking=yes pico '
set -eu
repo=$HOME/ROS2_RISCV
model=$(tr -d "\000" </proc/device-tree/model)
. /etc/os-release
arch=$(uname -m)
printf "K3_MODEL=%s\n" "$model"
printf "K3_OS_ID=%s\n" "${ID:-unset}"
printf "BIANBU_VERSION_ID=%s\n" "${VERSION_ID:-unset}"
printf "K3_ARCH=%s\n" "$arch"
case "$model" in
  *K3*Pico-ITX*|*K3*Pico\ ITX*) ;;
  *) exit 12 ;;
esac
[ "${ID:-}" = bianbu ]
command -v dpkg >/dev/null 2>&1
dpkg --compare-versions "${VERSION_ID:-0}" ge 4.0.1
[ "$arch" = riscv64 ]
remote_head=$(git -C "$repo" rev-parse HEAD)
remote_source_hash=$({
  git -C "$repo" ls-files --cached --others --exclude-standard -- setup_course_k3.sh src_k3_pico_itx |
    LC_ALL=C sort |
    while IFS= read -r path; do
      case "$path" in
        *.pyc|*/.venv/*|*/__pycache__/*|*/build/*|*/install/*|*/log/*) continue ;;
      esac
      [ -f "$repo/$path" ] || continue
      printf "%s %s\n" "$path" "$(sha256sum "$repo/$path" | cut -d " " -f 1)"
    done
} | sha256sum | cut -d " " -f 1)
printf "REMOTE_HEAD=%s\n" "$remote_head"
printf "REMOTE_SOURCE_SHA256=%s\n" "$remote_source_hash"
git -C "$repo" status --short
printf "REMOTE_STATUS_END\n"
')
remote_check_status=$?
set -e
if ((remote_check_status != 0)); then
  printf '%s\n' "$remote_state"
  printf 'REMOTE_CHECK_EXIT=%s\n' "$remote_check_status"
  printf 'SOURCE_SYNC_MATCH=false\n'
  exit 1
fi

printf '%s\n' "$remote_state"
remote_head=$(printf '%s\n' "$remote_state" | sed -n 's/^REMOTE_HEAD=//p' | tail -n 1)
remote_source_hash=$(printf '%s\n' "$remote_state" | sed -n 's/^REMOTE_SOURCE_SHA256=//p' | tail -n 1)

if [[ "$local_head" == "$remote_head" && "$local_source_hash" == "$remote_source_hash" ]]; then
  printf 'SOURCE_SYNC_MATCH=true\n'
else
  printf 'SOURCE_SYNC_MATCH=false\n'
  exit 1
fi
