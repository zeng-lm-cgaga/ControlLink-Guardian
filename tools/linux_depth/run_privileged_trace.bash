#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ ${CONTROL_LINK_TRACE_BOUNDED:-0} != 1 ]]; then
  export CONTROL_LINK_TRACE_BOUNDED=1
  exec timeout --signal=INT --kill-after=3s 20s bash "$0" "$@"
fi
BUILD=$(realpath "${1:?BPF/fixture build root required}")
OUT=${2:?new evidence directory required}
if [[ $EUID != 0 ]]; then
  echo 'Needs an authorized host administrator outside the restricted sandbox; no sudo/password escalation is performed.' >&2
  exit 2
fi
for event in sched/sched_wakeup sched/sched_switch sched/sched_process_exit raw_syscalls/sys_enter raw_syscalls/sys_exit; do
  if [[ ! -r /sys/kernel/tracing/events/$event/format && ! -r /sys/kernel/debug/tracing/events/$event/format ]]; then
    echo "Tracepoint unavailable/unreadable: $event (prepare tracefs outside this tool)" >&2
    exit 2
  fi
done
mkdir "$OUT"
OUT=$(realpath "$OUT")
python3 "$ROOT/tools/linux_depth/bpf_preflight.py" --output "$OUT/preflight.json"
target_pid=
loader_pid=
stop_children() {
  for owner in "$loader_pid" "$target_pid"; do
    if [[ -n $owner ]] && kill -0 "$owner" 2>/dev/null; then kill -INT "$owner"; fi
  done
  if [[ -n $loader_pid ]]; then wait "$loader_pid" || true; fi
  if [[ -n $target_pid ]]; then wait "$target_pid" || true; fi
}
trap stop_children EXIT
trap 'exit 130' INT TERM
"$BUILD/build-focused/controlled_workload" "$OUT/application.csv" >"$OUT/application.log" 2>&1 &
target_pid=$!
cat "/proc/$target_pid/status" >"$OUT/target-status.txt"
cat "/proc/$target_pid/stat" >"$OUT/target-stat.txt"
cat "/proc/$target_pid/maps" >"$OUT/target-maps.txt"
readlink "/proc/$target_pid/exe" >"$OUT/target-exe.txt"
sha256sum "$BUILD/build-focused/controlled_workload" "$BUILD/build-bpf/trace_loader" "$BUILD/build-bpf/scheduler.bpf.o" >"$OUT/elf.sha256"
"$BUILD/build-bpf/trace_loader" "$BUILD/build-bpf/scheduler.bpf.o" "$target_pid" 8 "$OUT/kernel.csv" >"$OUT/loader.log" 2>&1 &
loader_pid=$!
set +e
wait "$loader_pid"; loader_exit=$?; loader_pid=
wait "$target_pid"; application_exit=$?; target_pid=
set -e
printf '%s\n' "$loader_exit" >"$OUT/loader.exit"
printf '%s\n' "$application_exit" >"$OUT/application.exit"
if [[ $loader_exit != 0 || $application_exit != 0 ]]; then exit 1; fi
python3 "$ROOT/tools/linux_depth/analyze_bpf.py" "$OUT/kernel.csv" "$OUT/loader.log" \
  --application "$OUT/application.csv" --output "$OUT/summary.json"
