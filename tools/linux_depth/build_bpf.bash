#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
OUT=${1:?unique BPF build directory required}
mkdir -p "$OUT"
TC="$ROOT/reports/linux-depth-20261021/toolchain/root"
export LD_LIBRARY_PATH="$TC/usr/lib/llvm-14/lib:${LD_LIBRARY_PATH:-}"
"$TC/usr/bin/clang-14" --version
bpftool btf dump file /sys/kernel/btf/vmlinux format c > "$OUT/vmlinux.h"
"$TC/usr/bin/clang-14" -target bpf -D__TARGET_ARCH_x86 -O2 -g -Wall -Werror \
  -I"$OUT" -I"$TC/usr/include" -c "$ROOT/tools/linux_depth/scheduler.bpf.c" -o "$OUT/scheduler.bpf.o"
cc -std=c11 -O2 -g -Wall -Wextra -Werror -I"$TC/usr/include" \
  "$ROOT/tools/linux_depth/trace_loader.c" -o "$OUT/trace_loader" \
  "$TC/usr/lib/x86_64-linux-gnu/libbpf.a" -lelf -lz
