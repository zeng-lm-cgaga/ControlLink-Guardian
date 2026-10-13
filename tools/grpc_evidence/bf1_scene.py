#!/usr/bin/env python3
"""BF1单个串行业务组合，判据只读取协作停止后冻结的原始观察。"""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'someip_evidence'))
import judge
from run_scene import LiveCapture, marker, process_identity, runtime_identities, stop
from run_protocol_scene import evaluate_phases


def records(path):
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines()
            if line.startswith('{"event":') and line.endswith('}')]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', required=True)
    parser.add_argument('--report', required=True)
    parser.add_argument('--probe', required=True)
    parser.add_argument('--domain', required=True, type=int)
    arguments = parser.parse_args()
    root = Path(arguments.root).resolve(strict=True)
    report = Path(arguments.report).resolve()
    report.mkdir(parents=True, exist_ok=False)
    capture = report / 'capture'
    gates = report / 'preconditions'
    gates.mkdir()
    environment = os.environ.copy()
    environment.update(ROS_DOMAIN_ID=str(arguments.domain), ROS_LOCALHOST_ONLY='1',
                       RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS_LOG_DIR=str(report / 'ros-log'))
    for name in ('VSOMEIP_CONFIGURATION', 'VSOMEIP_CONFIGURATION_MODULE', 'VSOMEIP_APPLICATION_NAME',
                 'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
        environment.pop(name, None)
    client_environment = {key: value for key, value in environment.items()
                          if not key.startswith(('ROS_', 'AMENT_', 'COLCON_')) and key != 'RMW_IMPLEMENTATION'}
    processes, handles, commands = [], [], []
    events = {'started': marker(), 'scope': 'BF1_SINGLE_DEFAULT_50MS_COMBINATION',
              'period_ms': 50, 'keep_period_ms': 1000, 'domain': arguments.domain}
    result = 2

    def launch(label, command, env=environment):
        handle = (report / (label + '.log')).open('xb')
        handles.append(handle)
        process = subprocess.Popen(command, env=env, stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append((label, process))
        commands.append({'label': label, 'argv': command, 'pid': process.pid, 'started': marker(),
                         'environment': {name: env.get(name) for name in (
                             'ROS_DOMAIN_ID', 'AMENT_PREFIX_PATH', 'LD_LIBRARY_PATH', 'CMAKE_PREFIX_PATH')}})
        return process

    try:
        observer = launch('observer', [sys.executable, str(root / 'tools/someip_evidence/observer.py'),
            '--output-dir', str(capture), '--contract', str(root / 'config/common/gateway_contract.yaml'),
            '--can-interface', 'vcan0', '--max-seconds', '120'])
        deadline = time.monotonic() + 10
        while not (capture / 'capture-meta.json').exists():
            if observer.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError('Native observer initialization failed')
            time.sleep(0.02)
        events['observer_identity'] = process_identity(observer.pid)
        service = launch('launch', ['ros2', 'launch', 'control_link_bringup', 'adas_vcan_demo.launch.py',
            'planning_transport:=grpc', 'start_source:=false', 'can_interface:=vcan0'])
        time.sleep(2)
        if service.poll() is not None:
            raise RuntimeError('ROS assembly exited before client start')
        probe = launch('probe', [str(Path(arguments.probe).resolve(strict=True)),
            str(root / 'config/adas/protocol_transports.yaml'), str(gates)], client_environment)
        events['probe_identity'] = process_identity(probe.pid)
        live = LiveCapture(capture)
        mapping = judge.load_can_map((root / 'config/adas/can_signal_map.yaml').read_bytes())
        acknowledgements = {'before_rotation': 'rotate', 'after_rotation': 'pause',
                            'same_session_recovery': 'finish'}
        handled = set()
        deadline = time.monotonic() + 105
        while probe.poll() is None:
            if observer.poll() is not None or service.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError('Runtime/observer failed or bounded combination timed out')
            for record in records(report / 'probe.log'):
                if record['event'] != 'phase_begin' or record['phase'] not in acknowledgements:
                    continue
                phase = record['phase']
                if phase in handled:
                    continue
                if live.active(mapping, record['wall_clock_ns']) == record['gid']:
                    proof = {'marker': marker(), 'request': record, 'gid': record['gid'],
                             'basis': 'same GID source/Gateway sequence correlation plus CRC-valid nonzero CAN'}
                    judge.publish_json(gates / (acknowledgements[phase] + '.ok'), proof)
                    handled.add(phase)
                    events['probe_live_identity'] = process_identity(probe.pid)
            time.sleep(0.01)
        events['probe_exit'] = probe.wait()
        events['runtime_identities'] = runtime_identities(arguments.domain)
        if events['probe_exit'] != 0 or len(handled) != 3:
            raise RuntimeError('Probe or independent ACTIVE/CAN precondition failed')
        events['observer_stop_requested'] = marker()
        events['observer_wait_exit'] = stop(observer)
        if events['observer_wait_exit'] != 0:
            raise RuntimeError('Observer did not exit/flush cooperatively')
        frozen_records = records(report / 'probe.log')
        judge.publish_json(report / 'protocol-records.json', frozen_records)
        facts = evaluate_phases(report, frozen_records, capture, root, events['observer_wait_exit'])
        begins = {item['phase']: item for item in frozen_records if item['event'] == 'phase_begin'}
        before, after, recovered = (begins[name] for name in
            ('before_rotation', 'after_rotation', 'same_session_recovery'))
        if (before['gid'] == after['gid'] or before['session'] == after['session'] or
                before['domain'] >= after['domain']):
            raise RuntimeError('Rotation lacks new domain/session/publisher GID')
        if (after['gid'], after['session'], after['domain']) != (
                recovered['gid'], recovered['session'], recovered['domain']):
            raise RuntimeError('Command pause unexpectedly rotated the control session')
        if sum(item['event'] == 'keep' and item.get('phase') == 'keep_only'
               for item in frozen_records) < 2:
            raise RuntimeError('Pause lacks observed default-period Keep calls')
        judge.publish_json(report / 'business-facts.json', {'facts': facts,
            'rotation': {'before': before, 'after': after}, 'pause_same_session': True,
            'scope': 'VM_ONLY; not default ten-hour endurance or physical bus; no per-command execution ACK'})
        result = 0
    except Exception as failure:
        events['failure'] = str(failure)
        print(str(failure), file=sys.stderr, flush=True)
    finally:
        for label, process in reversed(processes):
            try:
                events[label + '_cleanup_exit'] = stop(process)
            except Exception as failure:
                events[label + '_cleanup_failure'] = str(failure)
                result = 2
        for handle in handles:
            try:
                handle.flush()
                os.fsync(handle.fileno())
                handle.close()
            except Exception as failure:
                events['write_failure'] = str(failure)
                result = 2
        events.update(ended=marker(), result=result, commands=commands)
        judge.publish_json(report / 'scenario.json', events)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
