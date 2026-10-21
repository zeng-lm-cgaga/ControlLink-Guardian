"""仅拥有一个 SocketCAN child；Graph/控制健康与恢复仍由原业务组件判断。"""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import threading
import time

import yaml


def load_policy(path):
    policy = yaml.safe_load(Path(path).read_text())
    expected = {'schema_version', 'max_restarts', 'restart_budget_ms', 'backoff_initial_ms',
                'backoff_max_ms', 'graceful_stop_ms', 'terminate_wait_ms'}
    if not isinstance(policy, dict) or set(policy) != expected:
        raise ValueError('supervisor policy fields must exactly match the schema')
    if any(type(value) is not int or value <= 0 for value in policy.values()):
        raise ValueError('supervisor policy values must be positive integers')
    if policy['schema_version'] != 1 or policy['max_restarts'] > 100:
        raise ValueError('unsupported supervisor schema/restart capacity')
    if policy['backoff_initial_ms'] > policy['backoff_max_ms'] or any(value > 3600000 for value in policy.values()):
        raise ValueError('invalid supervisor time budget')
    return policy


class AdapterSupervisor:
    """stop 可打断退避；旧child wait完成后才允许exec，重试预算从首次失败累计不清零。"""
    def __init__(self, command, policy, emit=None):
        self.command = command
        self.policy = policy
        self.stopped = threading.Event()
        self.child = None
        self.emit = emit or (lambda row: print(json.dumps(row, sort_keys=True), flush=True))

    def record(self, state, **details):
        self.emit({'monotonic_ns': time.monotonic_ns(), 'state': state, **details})

    def request_stop(self):
        self.stopped.set()

    def close_child(self):
        if self.child is None:
            return True
        graceful = True
        if self.child.poll() is None:
            for number, budget in ((signal.SIGINT, self.policy['graceful_stop_ms']),
                                   (signal.SIGTERM, self.policy['terminate_wait_ms']),
                                   (signal.SIGKILL, 1000)):
                try:
                    self.child.send_signal(number)
                except ProcessLookupError:
                    pass
                try:
                    self.child.wait(timeout=budget / 1000)
                    self.record('CHILD_REAPED', pid=self.child.pid, exit=self.child.returncode,
                                stop_signal=int(number), graceful=number == signal.SIGINT)
                    graceful = number == signal.SIGINT
                    break
                except subprocess.TimeoutExpired:
                    graceful = False
            if self.child.poll() is None:
                self.record('UNREAPED', pid=self.child.pid)
                raise RuntimeError('child did not exit; no replacement may be started')
        self.child.wait()
        self.child = None
        return graceful

    def run(self):
        restarts = 0
        first_failure = None
        backoff = self.policy['backoff_initial_ms'] / 1000
        try:
            while not self.stopped.is_set():
                try:
                    self.child = subprocess.Popen(self.command, close_fds=True, start_new_session=True)
                except OSError as failure:
                    self.record('EXEC_FAILED', error=str(failure))
                else:
                    stat = Path(f'/proc/{self.child.pid}/stat')
                    try:
                        starttime = int(stat.read_text().rsplit(')', 1)[1].split()[19])
                    except OSError:
                        starttime = None
                    self.record('CHILD_STARTED_NOT_BUSINESS_READY', pid=self.child.pid,
                                starttime=starttime, restarts=restarts, command=self.command)
                    while self.child.poll() is None and not self.stopped.wait(0.05):
                        pass
                    if self.stopped.is_set():
                        return 0 if self.close_child() else 3
                    code = self.child.wait()
                    self.record('CHILD_REAPED', pid=self.child.pid, exit=code, graceful=code == 0)
                    self.child = None
                    if code == 0:
                        self.record('NORMAL_EXIT_NO_RESTART')
                        return 0
                now = time.monotonic()
                if first_failure is None:
                    first_failure = now
                remaining = self.policy['restart_budget_ms'] / 1000 - (now - first_failure)
                if restarts >= self.policy['max_restarts'] or remaining <= backoff:
                    self.record('EXHAUSTED_MANUAL_INTERVENTION_REQUIRED', restarts=restarts)
                    return 75
                self.record('BACKOFF', seconds=backoff, restarts=restarts)
                if self.stopped.wait(backoff):
                    return 0
                restarts += 1
                backoff = min(backoff * 2, self.policy['backoff_max_ms'] / 1000)
            return 0
        finally:
            if self.child is not None:
                self.close_child()
            self.record('SUPERVISOR_STOPPED', restarts=restarts)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--policy', required=True)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command or not Path(command[0]).is_absolute() or Path(command[0]).name != 'socketcan_adapter_node':
        parser.error('only an absolute socketcan_adapter_node child is permitted')
    supervisor = AdapterSupervisor(command, load_policy(args.policy))
    previous = {number: signal.signal(number, lambda *unused: supervisor.request_stop())
                for number in (signal.SIGINT, signal.SIGTERM)}
    try:
        return supervisor.run()
    finally:
        for number, handler in previous.items():
            signal.signal(number, handler)


if __name__ == '__main__':
    raise SystemExit(main())
