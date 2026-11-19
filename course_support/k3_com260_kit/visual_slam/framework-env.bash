#!/usr/bin/env bash
set -eu
ws=$(cd "$(dirname "$0")" && pwd)
overlay=()
for dir in usr opt etc; do
  if [[ -d "$ws/root/$dir" ]]; then
    overlay+=(--overlay-src "/$dir" --overlay-src "$ws/root/$dir" --ro-overlay "/$dir")
  fi
done
exec bwrap --ro-bind / / --bind "$ws" "$ws" --proc /proc --dev /dev --tmpfs /tmp \
  "${overlay[@]}" --ro-bind "$ws/eigen/usr/include/eigen3" /usr/include/eigen3 \
  --ro-bind "$ws/eigen/usr/share/eigen3/cmake" /usr/share/eigen3/cmake \
  --setenv ROS_HOME "$ws/ros-home" --setenv XDG_CACHE_HOME "$ws/cache" --chdir "$ws" \
  bash -ec 'source /opt/ros/humble/setup.bash
    export CMAKE_PREFIX_PATH="$PWD/ceres:${CMAKE_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="$PWD/ceres/lib:${LD_LIBRARY_PATH:-}"
    export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
    export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=189
    unset CYCLONEDDS_URI
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    if [[ -f install/local_setup.bash ]]; then source install/local_setup.bash; fi
    exec "$@"' bash "$@"
