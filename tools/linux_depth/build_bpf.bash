#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
OUT=${1:?unique BPF build directory required}
LIBBPF=$(realpath "${2:?private libbpf 1.4.0 prefix required}")
VERSION=$(sed -n 's/^Version: //p' "$LIBBPF"/lib*/pkgconfig/libbpf.pc)
[[ $VERSION == 1.4.0 ]] || { echo "Expected fixed libbpf 1.4.0, got $VERSION" >&2; exit 2; }
ARCHIVE=$(find "$LIBBPF" -maxdepth 2 -name libbpf.a -type f)
[[ -f $ARCHIVE ]] || { echo 'Need exactly one private libbpf.a' >&2; exit 2; }
mkdir "$OUT"
TC="$ROOT/reports/linux-depth-20261021/toolchain/root"
export LD_LIBRARY_PATH="$TC/usr/lib/llvm-14/lib:${LD_LIBRARY_PATH:-}"
"$TC/usr/bin/clang-14" --version
bpftool btf dump file /sys/kernel/btf/vmlinux format c > "$OUT/vmlinux.h"
"$TC/usr/bin/clang-14" -target bpf -D__TARGET_ARCH_x86 -O2 -g -Wall -Werror \
  -I"$OUT" -I"$LIBBPF/include" -c "$ROOT/tools/linux_depth/scheduler.bpf.c" -o "$OUT/scheduler.bpf.o"
cc -std=c11 -O2 -g -Wall -Wextra -Werror -I"$LIBBPF/include" \
  "$ROOT/tools/linux_depth/trace_loader.c" -o "$OUT/trace_loader" \
  "$ARCHIVE" -lelf -lz
sha256sum "$ARCHIVE" "$OUT/trace_loader" "$OUT/scheduler.bpf.o" > "$OUT/link-identity.sha256"
"$OUT/trace_loader" --check-btf /sys/kernel/btf/vmlinux > "$OUT/loader-btf-check.json"
