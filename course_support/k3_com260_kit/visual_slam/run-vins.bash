#!/usr/bin/env bash
set -e
[[ ! -e out/vins ]] || { echo 'Move out/vins to keep the previous run before repeating.' >&2; exit 1; }
python3 tools/configure-vins.py .
mkdir -p out/vins
pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do
    result=0
    wait "$pid" || result=$?
    echo "$pid exit=$result" >>out/vins/exits.txt
    if (( result )); then status=1; fi
  done
}
trap 'status=$?; trap - EXIT; cleanup; exit "$status"' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
for package in feature_tracker vins_estimator pose_graph; do
  "install/$package/lib/$package/$package" --ros-args -r __ns:=/"$package" \
    -p config_file:="$PWD/sim-vins.yaml" -p vins_folder:="$PWD/src/vins/config_pkg" \
    -p support_file:="$PWD/src/vins/config_pkg/support_files" \
    </dev/null >"out/vins/$package.log" 2>&1 &
  pids+=($!)
done
python3 tools/replay-synthetic.py
