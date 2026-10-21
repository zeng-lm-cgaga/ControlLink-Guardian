#!/usr/bin/env python3
"""只读快照真实 owner 的进程/线程实例；不由 BPF 输出反推期望身份。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time


def snapshot(pid):
    directory = Path('/proc') / str(pid)
    raw = (directory / 'stat').read_text()
    start = int(raw.rsplit(')', 1)[1].split()[19])
    threads = {}
    for task in sorted((directory / 'task').iterdir()):
        text = (task / 'stat').read_text()
        threads[task.name] = {'starttime': int(text.rsplit(')', 1)[1].split()[19]), 'stat': text}
    exe = (directory / 'exe').resolve(strict=True)
    return {'pid': pid, 'starttime': start, 'clk_tck': os.sysconf('SC_CLK_TCK'),
            'captured_monotonic_ns': time.monotonic_ns(), 'threads': threads,
            'exe': str(exe), 'exe_sha256': hashlib.sha256(exe.read_bytes()).hexdigest(),
            'cmdline': (directory / 'cmdline').read_bytes().decode().split('\0'),
            'maps': (directory / 'maps').read_text(), 'stat': raw,
            'namespaces': {name: os.readlink(directory / 'ns' / name) for name in ('pid', 'time', 'user', 'net')}}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('pid', type=int)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    with args.output.open('x') as handle:
        json.dump(snapshot(args.pid), handle, indent=2, sort_keys=True)
        handle.write('\n')
