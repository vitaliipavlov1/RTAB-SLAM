#!/usr/bin/env bash
# Run the demo from the project root so config/rtabmap_minimal.ini is found.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPS="$ROOT/deps"
export LD_LIBRARY_PATH="$DEPS/opt/ros/humble/lib/x86_64-linux-gnu:$DEPS/usr/lib/x86_64-linux-gnu:$DEPS/realsense2/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
cd "$ROOT"
exec ./build/rtabmap_minimal "$@"
