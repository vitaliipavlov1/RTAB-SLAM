#!/usr/bin/env bash
# Single-job, niced build: this laptop swaps itself to death with -j4.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cmake -S "$ROOT" -B "$ROOT/build" -DCMAKE_BUILD_TYPE=Release
nice -n 10 cmake --build "$ROOT/build" -- -j1
echo "Built: $ROOT/build/rtabmap_minimal"
