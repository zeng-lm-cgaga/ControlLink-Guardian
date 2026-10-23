#!/usr/bin/env python3
"""仅运行一组受支持自routing-host订阅观察；复用原process owner收束。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/linux_depth'))
from capture_target import snapshot
from process_cleanup import stop_children


def save(path, value):
    with path.open('x') as handle:
        json.dump(value, handle, indent=2, sort_keys=True)
        handle.write('\n')


def set_network(path, network):
    document = json.loads(path.read_bytes())
    for service in document.get('services', []):
        keys = list(service)
        if 'events' in keys and 'eventgroups' in keys and keys.index('events') > keys.index('eventgroups'):
            raise ValueError('vsomeip 3.4.10 requires events before eventgroups to preserve reliability')
    document['network'] = network
    path.write_text(json.dumps(document) + '\n')


def run(output, binary, case):
    assert os.getuid() == os.geteuid() == 1000
    assert os.readlink('/proc/self/ns/net') != os.environ['CONTROL_LINK_PARENT_NET_NS']
    output.mkdir(parents=True, exist_ok=False)
    for name, arguments in [('link', ['link', 'show', 'lo']), ('address', ['address', 'show', 'lo']),
                            ('route', ['route', 'show'])]:
        save(output / ('network-' + name + '.json'), json.loads(subprocess.check_output(['ip', '-j', *arguments])))
    config = yaml.safe_load((ROOT / 'config/adas/protocol_transports.yaml').read_bytes())
    if case == 'nack':
        config['rate_limits'].update(per_caller_per_second=1, per_caller_burst=1, global_per_second=10, global_burst=10)
    policy = output / 'transport.yaml'
    policy.write_text(yaml.safe_dump(config, sort_keys=False))
    original = ROOT / 'reports/runtime-review-closure-20261021T163513Z-5976af9'
    generator = original / 'build-rate-probe/rate_limit_probe'
    subprocess.run([str(generator), 'configure', str(policy), str(output), '-', '-'], check=True)
    for role in ('server', 'client'):
        path = output / (role + '.json')
        set_network(path, 'cl_sub_' + str(os.getpid()) + '_' + role)
    children, logs, commands, identities, exit_codes = {}, {}, {}, {}, {}
    start = time.monotonic_ns()
    business_environment = dict(os.environ, LD_LIBRARY_PATH=str(ROOT / 'reports/2026-10-12_p1_f1/environment/vsomeip-install/lib'))
    failure = None
    try:
        commands['pcap'] = [sys.executable, str(original / 'packet_capture.py'), str(output / 'wire.pcap')]
        logs['pcap'] = (output / 'pcap.log').open('xb')
        children['pcap'] = subprocess.Popen(commands['pcap'], stdout=logs['pcap'], stderr=subprocess.STDOUT, start_new_session=True)
        ready_deadline = time.monotonic() + 2
        while not (output / 'wire.ready').exists():
            if children['pcap'].poll() is not None or time.monotonic() >= ready_deadline:
                raise RuntimeError('private packet capture not ready')
            time.sleep(.02)
        for role in ('client', 'server'):
            commands[role] = [str(binary), role + '-' + case, str(policy), str(output / (role + '.json')),
                              str(output / (role + '.jsonl')), str(output / 'done')]
            logs[role] = (output / (role + '.log')).open('xb')
            children[role] = subprocess.Popen(commands[role], stdout=logs[role], stderr=subprocess.STDOUT,
                                              start_new_session=True, env=business_environment)
            time.sleep(.3)
            if children[role].poll() is None:
                identities[role] = snapshot(children[role].pid)
                paths = {line.split()[-1] for line in identities[role]['maps'].splitlines()
                         if len(line.split()) >= 6 and line.split()[-1].startswith('/')}
                identities[role]['mapped_sha256'] = {
                    name: hashlib.sha256(Path(name).read_bytes()).hexdigest()
                    for name in sorted(paths) if not name.startswith('/dev/shm/') and Path(name).is_file()}
        for role in ('client', 'server'):
            exit_codes[role] = children[role].wait(timeout=18)
            if exit_codes[role]:
                raise RuntimeError('nonzero fixture: ' + repr(exit_codes))
        if any(exit_codes.values()):
            raise RuntimeError('nonzero fixture: ' + repr(exit_codes))
    except Exception as error:
        failure = type(error).__name__ + ': ' + str(error)
    finally:
        cleanup = stop_children(children)
        for handle in logs.values():
            handle.flush()
            os.fsync(handle.fileno())
            handle.close()
        save(output / 'result.json', {'failure': failure, 'commands': commands, 'identities': identities,
              'exit_codes': exit_codes, 'cleanup': cleanup, 'uid': os.getuid(), 'euid': os.geteuid(),
              'start_ns': start, 'flush_done_ns': time.monotonic_ns(),
              'config_sha256': hashlib.sha256(policy.read_bytes()).hexdigest(),
              'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
              'business_environment': {'LD_LIBRARY_PATH': business_environment['LD_LIBRARY_PATH']},
              'case': case,
              'scope': 'ROS-free actual TCP/SD, synthetic GID; safe observability only, strict NACK callback remains OPEN'})
    if failure or any(not row.get('exit_confirmed') or row.get('forced') or row.get('timeout') or row.get('errors')
                      for row in cleanup.values()):
        raise RuntimeError('cleanup/capture not closed: ' + str(failure))
    files = {str(path.relative_to(output)): {'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                                             'bytes': path.stat().st_size}
             for path in sorted(output.rglob('*')) if path.is_file()}
    save(output / 'frozen-inputs.json', {'files': files, 'scope': 'all owner writers waited/flushed before freezing'})


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--binary', required=True, type=Path)
    parser.add_argument('--case', required=True, choices=('ack', 'nack'))
    args = parser.parse_args()
    run(args.output.resolve(), args.binary.resolve(), args.case)
