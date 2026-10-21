#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
OUT=${1:?unique output directory required}
mkdir -p "$OUT"
OUT=$(realpath "$OUT")
set +u
source /opt/ros/humble/setup.bash
set -u
NATIVE="$ROOT/reports/valgrind-focused-20261013T142907Z-3199f9/install"
INTERFACES="$ROOT/reports/2026-10-12_p1_f1/environment/core-install/control_link_interfaces"
export MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1
export CMAKE_PREFIX_PATH="$NATIVE:$INTERFACES:${CMAKE_PREFIX_PATH:-}"
export AMENT_PREFIX_PATH="$NATIVE:$INTERFACES:${AMENT_PREFIX_PATH:-}"
for package in control_link_gateway control_link_adapters; do
  cmake -S "$ROOT/ros2_ws/src/$package" -B "$OUT/build/$package" \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX="$OUT/install" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBUILD_TESTING=ON \
    -DCONTROL_LINK_ENABLE_FUZZ_TARGETS=OFF -DCONTROL_LINK_ENABLE_EDGE_RUNTIME=OFF \
    -DCONTROL_LINK_ENABLE_SOMEIP=OFF -DCONTROL_LINK_ENABLE_GRPC=OFF \
    -DCONTROL_LINK_ENABLE_MQTT=OFF -DCONTROL_LINK_ENABLE_VCAN_TESTS=ON
  if [[ $package == control_link_gateway ]]; then
    targets=(control_link_gateway_node)
  else
    targets=(socketcan_adapter_node control_command_ingress_adapter_node twist_ingress_adapter_node test_adapter_core)
  fi
  cmake --build "$OUT/build/$package" --parallel 1 --target "${targets[@]}"
done
cmake -S "$ROOT/tools/linux_depth/peers" -B "$OUT/build/peers" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCONTROL_LINK_ADAPTER_LIBRARY="$OUT/build/control_link_adapters/libcontrol_link_adapters.so"
cmake --build "$OUT/build/peers" --parallel 1 --target vcan_vehicle_simulator
