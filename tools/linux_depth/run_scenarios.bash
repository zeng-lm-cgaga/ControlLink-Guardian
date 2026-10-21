#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
OUT=$(realpath "${1:?existing unique report directory}")
cd "$ROOT"
source tools/linux_depth/activate.bash "$OUT/runtime"
export ROS_DOMAIN_ID=86 ROS_LOG_DIR="$OUT/roslog"
ip link set lo up
ip link add vcan0 type vcan
ip link set vcan0 up
for index in 1 2 3; do
  python3 tools/linux_depth/run_scene.py --runtime "$OUT/runtime" --output "$OUT/scenes/final-off-$index" --seconds 20 --warmup 10
  python3 tools/linux_depth/run_scene.py --runtime "$OUT/runtime" --output "$OUT/scenes/final-on-$index" --seconds 20 --warmup 10 --timing
done
python3 tools/linux_depth/run_scene.py --runtime "$OUT/runtime" --output "$OUT/scenes/final-cpu-on" --seconds 20 --warmup 10 --timing --cpu-load
