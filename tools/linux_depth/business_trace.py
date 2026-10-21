"""短业务窗的观测握手与资源读数；不参与控制判据或恢复规则。"""
import json
import os
from pathlib import Path
import time


def publish(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.pending')
    if path.exists():
        raise FileExistsError(path)
    with temporary.open('x') as handle:
        json.dump(value, handle, sort_keys=True, indent=2)
        handle.write('\n')
        handle.flush()
        os.fsync(handle.fileno())
    os.rename(temporary, path)


class TraceGate:
    """业务保持运行，只有 broker 明确应答后才允许建立固定测量窗。"""
    def __init__(self, directory):
        self.directory = Path(directory)
        self.request = None
        self.reply = None

    def ready(self, targets, healthy, now):
        if self.request is None and healthy:
            self.request = {'requested_ns': now, 'uid': os.geteuid(), 'owner_pid': os.getpid(),
                            'targets': targets}
            publish(self.directory / 'request.json', self.request)
        if self.request is None:
            return False
        if now - self.request['requested_ns'] > 20000000000 and self.reply is None:
            raise TimeoutError('bounded BPF broker readiness timeout')
        path = self.directory / 'ready.json'
        if self.reply is None and path.exists():
            self.reply = json.loads(path.read_bytes())
            if self.reply.get('requested_ns') != self.request['requested_ns'] or not self.reply.get('ready'):
                raise ValueError('trace readiness request mismatch/failure')
            if self.reply.get('mode') not in ('off', 'on'):
                raise ValueError('unknown BPF mode')
            if self.reply.get('targets') != targets:
                raise ValueError('trace target instance changed')
        return self.reply is not None and healthy


def resource_sample(pid, role):
    begin = time.monotonic_ns()
    directory = Path('/proc') / str(pid)
    fields = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
    status = {line.split(':', 1)[0]: line.split(':', 1)[1].strip()
              for line in (directory / 'status').read_text().splitlines() if ':' in line}
    return {'role': role, 'pid': pid, 'starttime': int(fields[19]),
            'cpu_ticks': int(fields[11]) + int(fields[12]),
            'rss_kib': int(status['VmRSS'].split()[0]), 'threads': int(status['Threads']),
            'monotonic_ns': (begin + time.monotonic_ns()) // 2}
