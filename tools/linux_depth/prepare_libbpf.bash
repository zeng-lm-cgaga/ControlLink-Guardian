#!/usr/bin/env bash
set -euo pipefail
OUT=${1:?new private toolchain directory required}
mkdir "$OUT"
OUT=$(realpath "$OUT")
ARCHIVE="$OUT/libbpf-v1.4.0.tar.gz"
curl -fL --retry 2 https://codeload.github.com/libbpf/libbpf/tar.gz/refs/tags/v1.4.0 -o "$ARCHIVE"
printf '%s  %s\n' 4271dfd51f59d23c03d9ae62d8a92622af0f5216a620a7a666c4975c3c7cda6e "$ARCHIVE" | sha256sum -c -
tar -xzf "$ARCHIVE" -C "$OUT"
export MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1
make -C "$OUT/libbpf-1.4.0/src" -j1 BUILD_STATIC_ONLY=1 OBJDIR="$OUT/objects" PREFIX="$OUT/prefix" install
sha256sum "$ARCHIVE" "$OUT"/prefix/lib*/libbpf.a > "$OUT/toolchain.sha256"
