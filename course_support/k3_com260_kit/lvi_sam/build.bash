#!/usr/bin/env bash
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
ws="$HOME/ros2_course_com260_ch21_lvi"
if [[ -e $ws && ! -f "$ws/.com260-lvi" ]]; then
  echo "Refusing unmarked workspace: $ws" >&2
  exit 1
fi
mkdir -p "$ws"/{debs/partial,eigen,root,src/lvi_sam}
touch "$ws/.com260-lvi"
cd "$ws"
exec 9>.build.lock
flock -n 9 || { echo 'Another LVI-SAM build is in progress.' >&2; exit 1; }
tmp=$(mktemp -d "$ws/.prepare.XXXXXX")
trap 'rm -rf -- "$tmp"' EXIT

revision=12187f80d8c955241d0f28f03d567a5092af2703
archive=lvi-sam-$revision.tar.gz
checksum=7432937d733387639e7ac267c104f8b85589f68cfb102197a3f30f67537e67a6
source_file="$ws/$archive"
if [[ ! -f $source_file ]]; then
  if [[ -f "$HOME/Downloads/$archive" ]]; then
    source_file="$HOME/Downloads/$archive"
  else
    source_file="$tmp/source.tar.gz"
    curl --fail --location --silent --show-error --retry 3 \
      "https://api.github.com/repos/M-MohsinKhan/LVI-SAM-ROS2/tarball/$revision" -o "$source_file"
  fi
fi
printf '%s  %s\n' "$checksum" "$source_file" | sha256sum -c -
if [[ $source_file == "$tmp/source.tar.gz" ]]; then
  mv "$source_file" "$archive"
  source_file="$ws/$archive"
fi
tar -xzf "$source_file" --strip-components=1 -C "$tmp"
patch --batch --ignore-whitespace --fuzz=0 -s -d "$tmp" -p1 -i "$here/humble-handheld.patch"
rsync -r --checksum "$tmp/" src/lvi_sam/

# Download the packages selected for this Bianbu installation without changing its package database.
bwrap --ro-bind / / --bind "$ws" "$ws" --dev /dev --proc /proc --tmpfs /tmp \
  apt-get -o Debug::NoLocking=1 -o Dir::Cache::archives="$ws/debs" -o Dir::Log="$ws" \
  --download-only --no-install-recommends --no-remove -y install libgtsam-dev libceres-dev \
  ros-humble-pcl-ros ros-humble-tf2-eigen ros-humble-xacro
for deb in debs/*.deb; do
  [[ ! -f $deb ]] || dpkg-deb -x "$deb" root
done
# GTSAM 4.2 on Bianbu uses Eigen 3.4. The default board headers identify as Eigen 5.
(cd eigen && apt-get download libeigen3-dev=3.4.0-5 && dpkg-deb -x libeigen3-dev_3.4.0-5_all.deb .)
lib=root/usr/lib/riscv64-linux-gnu
mkdir -p "$lib/pkgconfig"
for name in blas lapack; do ln -sfn "$name/lib$name.so" "$lib/lib$name.so"; done
ln -sfn openblas-pthread/libopenblas.so.0 "$lib/libopenblas.so.0"
ln -sfn openmpi/lib/libmpi.so "$lib/libmpi.so"
for lang in c cxx; do ln -sfn "ompi-$lang.pc" "$lib/pkgconfig/mpi-$lang.pc"; done

install -m 755 "$here/run.bash" run
MAKEFLAGS=-j4 CMAKE_BUILD_PARALLEL_LEVEL=4 ./run python3 -m colcon build \
  --base-paths src/lvi_sam --packages-select lvi_sam --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=MinSizeRel '-DCMAKE_CXX_FLAGS_MINSIZEREL=-O1 -g0 -DNDEBUG'
