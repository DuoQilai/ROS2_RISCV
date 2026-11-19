#!/usr/bin/env bash
# Enter the private Humble library environment; only the LVI-SAM directory is writable.
set -eu
ws=$(cd "$(dirname "$0")" && pwd)
if [[ ${1:-} == ros2 ]]; then
  source "$HOME/.config/ros2-course-com260-ch20-ch21/vision-dds.bash"
fi
if [[ ${1:-} == ros2 && ${2:-} == launch ]]; then
  # Let ros2 launch stop its nodes once, even when Ctrl+C reaches the whole terminal group.
  trap '' INT
  set -- env --default-signal=INT python3 -c '
import signal, subprocess, sys
launch = subprocess.Popen(sys.argv[1:], stdin=subprocess.DEVNULL, start_new_session=True)
signal.signal(signal.SIGINT, lambda *_: launch.send_signal(signal.SIGINT))
sys.exit(launch.wait())
' "$@"
fi
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
    if [[ -r install/local_setup.bash ]]; then source install/local_setup.bash; fi
    export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
    exec "$@"' bash "$@"
