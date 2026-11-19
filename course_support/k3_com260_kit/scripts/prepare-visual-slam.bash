#!/usr/bin/env bash
# Build the optional visual-inertial SLAM dependency in the user's directory.
set -Eeuo pipefail
die() { printf 'Error: %s\n' "$*" >&2; exit 1; }
[[ $(uname -m) == riscv64 && $EUID -ne 0 ]] || die 'Run as the COM260 course user.'
root="$HOME/ros2_course_com260_ch21_slam"
[[ ! -e $root || -f $root/.com260-visual-slam ]] || die "Refusing unmarked directory: $root"
mkdir -p "$root/debs" "$root/sysroot"
touch "$root/.com260-visual-slam"
cd "$root"
for source in \
  'ORB_SLAM3 https://github.com/UZ-SLAMLab/ORB_SLAM3.git 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4' \
  'Pangolin https://github.com/stevenlovegrove/Pangolin.git aff6883c83f3fd7e8268a9715e84266c42e2efe3'; do
  read -r directory url revision <<< "$source"
  if [[ ! -d $directory ]]; then
    git clone --no-checkout --depth 1 "$url" "$directory"
    git -C "$directory" fetch --depth 1 origin "$revision"
    git -C "$directory" checkout --detach "$revision"
  fi
  [[ $(git -C "$directory" rev-parse HEAD) == "$revision" ]] || die "Unexpected source revision: $directory"
done
(
  cd debs
  apt-get download libboost1.90-dev=1.90.0-6ubuntu1 \
    libboost-serialization1.90-dev=1.90.0-6ubuntu1 libboost-serialization1.90.0=1.90.0-6ubuntu1 \
    libssl-dev=3.5.5-1ubuntu3bb1 libssl3t64=3.5.5-1ubuntu3bb1 \
    libegl-dev=1.7.0-3 libx11-dev=2:1.8.13-1 libxext-dev=2:1.3.4-1build3
  for package in ./*.deb; do dpkg-deb -x "$package" ../sysroot; done
)
cat > env.bash <<'ENV'
export ORB_SLAM3_ROOT="$HOME/ros2_course_com260_ch21_slam/ORB_SLAM3"
export CMAKE_PREFIX_PATH="$HOME/ros2_course_com260_ch21_slam/install:$HOME/ros2_course_com260_ch21_slam/sysroot/usr:${CMAKE_PREFIX_PATH:-}"
export CPLUS_INCLUDE_PATH="$HOME/ros2_course_com260_ch21_slam/sysroot/usr/include:$HOME/ros2_course_com260_ch21_slam/sysroot/usr/include/riscv64-linux-gnu:${CPLUS_INCLUDE_PATH:-}"
export LIBRARY_PATH="$HOME/ros2_course_com260_ch21_slam/sysroot/usr/lib/riscv64-linux-gnu:${LIBRARY_PATH:-}"
export LD_LIBRARY_PATH="$HOME/ros2_course_com260_ch21_slam/sysroot/usr/lib/riscv64-linux-gnu:$HOME/ros2_course_com260_ch21_slam/install/lib:${LD_LIBRARY_PATH:-}"
ENV
set +u
source env.bash
set -u
prefix="$root/sysroot/usr"
cmake -S Pangolin -B Pangolin/build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$root/install" -DCMAKE_PREFIX_PATH="$prefix" \
  -DCMAKE_CXX_FLAGS="-include cstdint" -DBUILD_TOOLS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF \
  -DCMAKE_DISABLE_FIND_PACKAGE_FFMPEG=TRUE -DBUILD_PANGOLIN_PYTHON=OFF
cmake --build Pangolin/build -j4
cmake --install Pangolin/build
python3 - <<'PATCH'
from pathlib import Path
root=Path('ORB_SLAM3')
for p in [root/'CMakeLists.txt',root/'Thirdparty/DBoW2/CMakeLists.txt',root/'Thirdparty/g2o/CMakeLists.txt']:
 original=p.read_text()
 text=original.replace('-march=native','').replace('-std=c++11','-std=c++17')
 if text != original: p.write_text(text)
p=root/'include/LoopClosing.h'
original=p.read_text()
text=original.replace('bool mnFullBAIdx;', 'int mnFullBAIdx;')
if text != original: p.write_text(text)
PATCH
cmake -S ORB_SLAM3/Thirdparty/DBoW2 -B ORB_SLAM3/Thirdparty/DBoW2/build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_CXX_STANDARD=17
cmake --build ORB_SLAM3/Thirdparty/DBoW2/build -j4
cmake -S ORB_SLAM3 -B ORB_SLAM3/build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_CXX_STANDARD=17 \
  -DG2O_EIGEN3_INCLUDE=/usr/include/eigen3 \
  -DCMAKE_PREFIX_PATH="$root/install;$prefix" \
  -DCMAKE_SHARED_LINKER_FLAGS="-L$prefix/lib/riscv64-linux-gnu -Wl,-rpath,$prefix/lib/riscv64-linux-gnu"
cmake --build ORB_SLAM3/build --target g2o -j4
cmake --build ORB_SLAM3/build --target ORB_SLAM3 mono_inertial_euroc rgbd_tum -j4
if [[ ! -f ORB_SLAM3/Vocabulary/ORBvoc.txt ]]; then
  tar -xzf ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz -C ORB_SLAM3/Vocabulary
fi
