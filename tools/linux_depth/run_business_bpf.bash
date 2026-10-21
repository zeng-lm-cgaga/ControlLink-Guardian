#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ $EUID != 0 ]]; then
  echo 'Run once from the authorized host administrator terminal; this tool never invokes sudo.' >&2
  exit 2
fi
OUT=${1:?new absolute evidence directory required}
RUNTIME=${2:?explicit verified runtime directory required}
exec timeout --signal=INT --kill-after=90s 360s unshare --net -- \
  python3 "$ROOT/tools/linux_depth/run_business_bpf.py" --output "$OUT" --runtime "$RUNTIME" --user zlm
