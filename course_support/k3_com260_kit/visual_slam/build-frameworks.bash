#!/usr/bin/env bash
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
ws="$HOME/ros2_course_com260_ch21_frameworks"
[[ $(uname -m) == riscv64 ]] || { echo 'Run on COM260.' >&2; exit 1; }
[[ ! -e "$ws" || -f "$ws/.com260-frameworks" ]] || { echo "Unmarked workspace: $ws" >&2; exit 1; }
mkdir -p "$ws"/{src,debs/partial,root,eigen,tools}
touch "$ws/.com260-frameworks"
cd "$ws"
exec 9>.build.lock
flock -n 9 || { echo 'Build already running.' >&2; exit 1; }
for spec in \
  'dso JakobEngel/dso 7b0c99f01d238f801c625beaff90240bcb007198' \
  'vins dongbo19/VINS-MONO-ROS2 4c3cf08fa5d9b57f3be9bd49241915e968ee2e1e' \
  'ceres ceres-solver/ceres-solver 2.1.0'; do
  read -r name repo revision <<<"$spec"
  archive="$name-$revision.tar.gz"
  if [[ ! -f "$archive" ]]; then
    if [[ -f "$HOME/Downloads/$archive" ]]; then
      cp "$HOME/Downloads/$archive" "$archive"
    else
      curl -fL --retry 3 "https://api.github.com/repos/$repo/tarball/$revision" -o "$archive.part"
      mv "$archive.part" "$archive"
    fi
  fi
  temp=$(mktemp -d "$ws/.source.XXXXXX")
  tar -xzf "$archive" --strip-components=1 -C "$temp"
  if [[ $name == dso ]]; then patch --batch --fuzz=0 -s -d "$temp" -p1 -i "$here/dso-riscv.patch"; fi
  if [[ $name == vins ]]; then patch --batch --fuzz=0 -s -d "$temp" -p1 -i "$here/vins-humble.patch"; fi
  mkdir -p "src/$name"
  rsync -r --checksum "$temp/" "src/$name/"
  rm -rf "$temp"
done
bwrap --ro-bind / / --bind "$ws" "$ws" --dev /dev --proc /proc --tmpfs /tmp \
  apt-get -o Debug::NoLocking=1 -o Dir::Cache::archives="$ws/debs" -o Dir::Log="$ws" \
  --download-only --no-install-recommends --no-remove -y install \
  libsuitesparse-dev libboost-thread-dev libboost-filesystem-dev libboost-program-options-dev libzip-dev libsimde-dev
for deb in debs/*.deb; do [[ ! -f "$deb" ]] || dpkg-deb -x "$deb" root; done
(cd eigen && apt-get download libeigen3-dev=3.4.0-5 && dpkg-deb -x libeigen3-dev_3.4.0-5_all.deb .)
install -m 755 "$here/framework-env.bash" run
cp "$here"/{configure-vins.py,replay-synthetic.py,run-vins.bash,evaluate-synthetic.py} tools/
./run cmake -S src/ceres -B ceres-build -DCMAKE_BUILD_TYPE=Release \
  '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -g0 -DNDEBUG' -DCMAKE_INSTALL_PREFIX="$ws/ceres" \
  -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DMINIGLOG=ON \
  -DSUITESPARSE=OFF -DCXSPARSE=OFF -DLAPACK=OFF -DEIGENSPARSE=ON -DSCHUR_SPECIALIZATIONS=OFF
./run cmake --build ceres-build -j4
./run cmake --install ceres-build
./run cmake -S src/dso -B dso-build -DCMAKE_DISABLE_FIND_PACKAGE_Pangolin=ON
./run cmake --build dso-build -j4
MAKEFLAGS=-j4 CMAKE_BUILD_PARALLEL_LEVEL=4 ./run python3 -m colcon build \
  --base-paths src/vins --packages-up-to feature_tracker vins_estimator pose_graph config_pkg \
  --executor sequential --cmake-args -DCeres_DIR="$ws/ceres/lib/cmake/Ceres" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -g0 -DNDEBUG'
