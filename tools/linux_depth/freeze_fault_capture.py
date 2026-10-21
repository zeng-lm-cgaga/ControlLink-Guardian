#!/usr/bin/env python3
"""owner停止/wait后冻结功能自有日志；不把轮转快照当作完整决策回放。"""
import argparse
import hashlib
import json
from pathlib import Path


def freeze(directory, output, pid, starttime):
    directory = directory.resolve()
    if output.resolve().is_relative_to(directory):
        raise ValueError('write manifest outside the capture-owned budget directory')
    proc = Path('/proc') / str(pid) / 'stat'
    try:
        if int(proc.read_text().rsplit(')', 1)[1].split()[19]) == starttime:
            raise ValueError('capture owner is still present; stop/wait/flush first')
    except FileNotFoundError:
        pass
    status = json.loads((directory / 'capture-status.json').read_text())
    if not status.get('closed') or status.get('pid') != pid or status.get('starttime') != starttime or status.get('io_errors'):
        raise ValueError('missing/mismatched/error final writer status')
    files = sorted(directory.iterdir())
    artifacts = []
    for path in files:
        if path.is_symlink() or not path.is_file() or (path.name != 'capture-status.json' and
                not (path.name.startswith(('continuous-', 'fault-')) and path.suffix == '.jsonl')):
            raise ValueError('unexpected file in capture-owned directory')
        before = path.stat()
        checksum = hashlib.sha256()
        with path.open('rb') as source:
            for chunk in iter(lambda: source.read(1048576), b''):
                checksum.update(chunk)
        after = path.stat()
        if (before.st_size, before.st_mtime_ns, before.st_ino) != (after.st_size, after.st_mtime_ns, after.st_ino):
            raise ValueError('capture changed during hash')
        artifacts.append({'path': str(path), 'bytes': after.st_size, 'sha256': checksum.hexdigest()})
    if sorted(directory.iterdir()) != files:
        raise ValueError('capture file population changed during hash')
    with output.open('x') as destination:
        json.dump({'schema_version': 1, 'pid': pid, 'starttime': starttime, 'replay_complete': False,
                   'limits': 'post-exit immutable partial snapshots; missing/rotated history is not replayable',
                   'status': status, 'artifacts': artifacts}, destination, indent=2, sort_keys=True)
        destination.write('\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--pid', required=True, type=int)
    parser.add_argument('--starttime', required=True, type=int)
    arguments = parser.parse_args()
    freeze(arguments.directory, arguments.output, arguments.pid, arguments.starttime)
