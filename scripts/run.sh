#!/usr/bin/env bash
# Run from the project root so config/rtab_slam.ini is found.
#
# Artifacts go to runs/<timestamp>.{db,pcd,txt} unless --db/--cloud say otherwise,
# which keeps the repository root free of run output. The binary itself still
# writes to the current directory when it is started directly.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPS="$ROOT/deps"
export LD_LIBRARY_PATH="$DEPS/opt/ros/${ROS_DISTRO:-jazzy}/lib/x86_64-linux-gnu:$DEPS/usr/lib/x86_64-linux-gnu:$DEPS/realsense2/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
cd "$ROOT"
if [ ! -x ./build/rtab_slam ]; then
  echo "Not built yet: run ./scripts/build.sh first." >&2
  exit 1
fi

args=("$@")
run="runs/$(date +%Y-%m-%d_%H%M%S)"
mkdir -p runs
[[ " $* " == *" --db "* ]] || args+=(--db "$run.db")
[[ " $* " == *" --cloud "* ]] || args+=(--cloud "$run.pcd")
[[ " $* " == *" --traj "* ]] || args+=(--traj "$run""_trajectory.txt")

exec ./build/rtab_slam "${args[@]}"
