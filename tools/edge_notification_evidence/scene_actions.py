"""仅驱动自有夹具的固定故障动作；状态/恢复判据仍取实际 Gateway 与 CAN。"""
from pathlib import Path
import subprocess
import time


class EdgeActions:
    def __init__(self, output, mode, faults):
        self.output, self.mode, self.faults = Path(output), mode, faults
        self.command = 'cl_v05_command_' + str(time.monotonic_ns())
        self.feedback = self.command + '_feedback'
        self.gate = self.output / 'edge-control.txt'
        self.set_phase('run')
        self.done = set()

    def set_phase(self, phase):
        temporary = self.gate.with_suffix('.partial')
        temporary.write_text(phase + '\n')
        temporary.replace(self.gate)

    def bridge_arguments(self):
        return ['-p', 'command_channel:=' + self.command, '-p', 'feedback_channel:=' + self.feedback,
                '-p', 'wait_mode:=' + self.mode]

    def client_command(self, path):
        return [str(path), self.command, self.feedback, self.mode, str(self.gate)]

    def tick(self, window, events, children, logs, identity, stop_children):
        if not self.faults or not window:
            return
        offset = time.monotonic_ns() - window['start_ns']
        for name, seconds, phase in [('pause', 2, 'pause'), ('resume', 3, 'run'), ('command_replace', 6, 'reopen')]:
            if offset >= seconds * 1000000000 and name not in self.done:
                self.set_phase(phase)
                events.setdefault('edge_actions', {})[name] = {'monotonic_ns': time.monotonic_ns(), 'phase': phase}
                self.done.add(name)
        if offset >= 30 * 1000000000 and 'feedback_replace' not in self.done:
            role = 'planning_ingress'
            before = identity(children[role].pid)
            closed = stop_children({role: children[role]})[role]
            event = {'requested_ns': time.monotonic_ns(), 'before': before, 'stop': closed}
            events.setdefault('edge_actions', {})['feedback_replace'] = event
            if not closed['exit_confirmed'] or closed['exit'] or closed.get('timeout'):
                raise RuntimeError('old Edge bridge did not stop cleanly')
            children[role] = subprocess.Popen(events['commands'][role], stdout=logs[role], stderr=subprocess.STDOUT,
                                              start_new_session=True)
            event['after'] = identity(children[role].pid)
            event['started_ns'] = time.monotonic_ns()
            self.done.add('feedback_replace')
        replacement = events.get('edge_actions', {}).get('feedback_replace')
        if replacement and 'after_running' not in replacement and time.monotonic_ns() - replacement['started_ns'] >= 1000000000:
            replacement['after_running'] = identity(children['planning_ingress'].pid)
