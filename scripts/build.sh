#!/usr/bin/env bash
# Single-job build under nice: two cores are enough to swap a laptop out with -j4.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cmake -S "$ROOT" -B "$ROOT/build" -DCMAKE_BUILD_TYPE=Release
nice -n 10 cmake --build "$ROOT/build" -- -j1
echo "Built: $ROOT/build/rtab_slam"
