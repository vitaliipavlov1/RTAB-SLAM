#!/usr/bin/env bash
# Fetch RTAB-Map (+ g2o/GTSAM) and librealsense2 into ./deps without root.
#
# If you do have root, this script is not needed at all:
#     sudo apt install ros-humble-rtabmap librealsense2-dev
# Everything below is just the rootless equivalent: the same .deb packages are
# downloaded with a user-local apt state and unpacked into ./deps.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPS="$ROOT/deps"
WORK="$ROOT/.apt"

if [ -e /opt/ros/humble/lib/x86_64-linux-gnu/librtabmap_core.so ]; then
  echo "System RTAB-Map found in /opt/ros/humble, nothing to fetch."
elif [ -e "$DEPS/opt/ros/humble/lib/x86_64-linux-gnu/librtabmap_core.so" ]; then
  echo "RTAB-Map already unpacked in $DEPS, nothing to fetch."
else
  echo "== fetching RTAB-Map packages into $DEPS"
  mkdir -p "$WORK"/{lists/partial,cache/archives/partial} "$DEPS"
  APT_OPTS=(-o "Dir::State::Lists=$WORK/lists"
            -o "Dir::Cache=$WORK/cache"
            -o "Dir::Cache::archives=$WORK/cache/archives"
            -o Debug::NoLocking=1)
  apt-get "${APT_OPTS[@]}" update
  apt-get "${APT_OPTS[@]}" install -y --download-only ros-humble-rtabmap
  for deb in "$WORK"/cache/archives/*.deb; do dpkg-deb -x "$deb" "$DEPS"; done
  rm -rf "$WORK"
fi

# librealsense2: use the system one if present, otherwise copy an existing
# local SDK prefix (headers + libs only, no tools).
RS_SRC="${RS_SRC:-$HOME/projects/3d_vision/third_party/librealsense}"
if pkg-config --exists realsense2 2>/dev/null; then
  echo "System librealsense2 found, nothing to copy."
elif [ -d "$DEPS/realsense2/include/librealsense2" ]; then
  echo "librealsense2 already in $DEPS/realsense2."
elif [ -d "$RS_SRC/include/librealsense2" ]; then
  echo "== copying librealsense2 from $RS_SRC (read-only source)"
  mkdir -p "$DEPS/realsense2"
  cp -r "$RS_SRC/include" "$RS_SRC/lib" "$DEPS/realsense2/"
else
  echo "librealsense2 not found. Install it (sudo apt install librealsense2-dev)"
  echo "or point RS_SRC= at an existing SDK install prefix." >&2
  exit 1
fi

echo "Done. Dependencies in $DEPS"
