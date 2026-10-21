#!/usr/bin/env python3
"""一次串行 off/on；root 仅装配私有 netns/观测器，所有 ROS 子进程先降权。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import pwd
import shlex
import shutil
import signal
import subprocess
import time
import traceback

from analyze import encoded, freeze
from business_trace import publish, resource_sample
from capture_target import snapshot

ROOT = Path(__file__).resolve().parents[2]
RUNTIME = ROOT / 'reports/linux-depth-20261021/remote/runtime'
BPF = ROOT / 'reports/linux-depth-review-fixes-20261021/build-bpf'
ROLES = ('gateway', 'can_adapter')


def runtime_identity(pid):
    fact = snapshot(pid)
    paths = {line.split()[-1] for line in fact['maps'].splitlines()
             if len(line.split()) >= 6 and line.split()[-1].startswith('/')}
    fact['mapped_sha256'] = {name: hashlib.sha256(Path(name).read_bytes()).hexdigest()
                             for name in sorted(paths) if not name.startswith('/dev/shm/') and Path(name).is_file()}
    return fact


def stop(child):
    result = {'pid': child.pid, 'requested_ns': time.monotonic_ns(), 'forced': False}
    for number, seconds in ((signal.SIGINT, 25), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
        if child.poll() is not None:
            break
        os.killpg(child.pid, number)
        try:
            child.wait(timeout=seconds)
        except subprocess.TimeoutExpired:
            result['forced'] = True
    result.update(exit=child.wait(timeout=1), waited_ns=time.monotonic_ns())
    return result


def checked_target(expected, uid):
    actual = snapshot(expected['pid'])
    status = Path('/proc', str(expected['pid']), 'status').read_text()
    users = next(line.split()[1:] for line in status.splitlines() if line.startswith('Uid:'))
    if (actual['starttime'], actual['exe_sha256']) != (expected['starttime'], expected['exe_sha256']):
        raise ValueError('target instance/executable drift')
    if uid == 0 or any(int(value) != uid for value in users):
        raise ValueError('business target must have the ordinary user UID')
    for name in ('pid', 'time', 'user', 'net'):
        if actual['namespaces'][name] != os.readlink('/proc/self/ns/' + name):
            raise ValueError('business/tracer namespace mismatch: ' + name)
    return actual


def link_facts(pid):
    facts = []
    for path in Path('/proc', str(pid), 'fd').iterdir():
        try:
            if any(name in os.readlink(path) for name in ('bpf-link', 'bpf_link')):
                facts.append({'fd': int(path.name), 'fdinfo': Path('/proc', str(pid), 'fdinfo', path.name).read_text()})
        except FileNotFoundError:
            continue
    return facts


def thread_observation(target):
    directory = Path('/proc', str(target['pid']))
    fields = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
    if int(fields[19]) != target['starttime']:
        raise ValueError('target process instance reused')
    threads = {}
    for path in (directory / 'task').iterdir():
        try:
            threads[path.name] = int((path / 'stat').read_text().rsplit(')', 1)[1].split()[19])
        except FileNotFoundError:
            continue
    return {'monotonic_ns': time.monotonic_ns(), 'threads': threads}


def frozen_case(case):
    files = {}
    for path in sorted(case.rglob('*')):
        if path.is_file():
            files[str(path.relative_to(case))] = {'sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'size': path.stat().st_size}
    publish(case / 'capture-freeze.json', {'schema': 1, 'files': files})


def run_case(base, mode, user, domain):
    case = base / mode
    case.mkdir()
    control = case / 'control'
    control.mkdir()
    for path in (case, control):
        os.chown(path, user.pw_uid, user.pw_gid)
    bpf = case / 'bpf'
    bpf.mkdir()
    env = dict(os.environ, HOME=user.pw_dir, USER=user.pw_name, LOGNAME=user.pw_name,
               ROS_DOMAIN_ID=str(domain), ROS_LOG_DIR=str(case / 'roslog'), PYTHONDONTWRITEBYTECODE='1')
    script = 'source ' + shlex.quote(str(ROOT / 'tools/linux_depth/activate.bash')) + ' ' + shlex.quote(str(RUNTIME))
    script += '; exec python3 ' + shlex.join([str(ROOT / 'tools/linux_depth/run_scene.py'), '--runtime', str(RUNTIME),
              '--output', str(case / 'scene'), '--seconds', '20', '--warmup', '10', '--timing', '--bpf-control', str(control)])
    command = ['runuser', '-u', user.pw_name, '--', 'bash', '-e', '-c', script]
    record = {'mode': mode, 'business_uid': user.pw_uid, 'business_command': command,
              'environment': {key: env[key] for key in ('ROS_DOMAIN_ID', 'ROS_LOG_DIR', 'RMW_IMPLEMENTATION') if key in env},
              'clock': 'CLOCK_MONOTONIC; no new time/PID/user namespace', 'errors': [], 'targets': {}, 'tracers': {}, 'exits': {},
              'thread_observations': {role: [] for role in ROLES}}
    children, logs = {}, []
    resource = (bpf / 'resource.jsonl').open('x')
    started, next_sample = time.monotonic_ns(), 0
    trace_stopped = False
    try:
        log = (case / 'business.log').open('xb')
        logs.append(log)
        children['business'] = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        while children['business'].poll() is None:
            now = time.monotonic_ns()
            if now - started > 150000000000:
                raise TimeoutError('bounded business case exhausted')
            if shutil.disk_usage(base).free < 4 * 1024**3 or sum(path.stat().st_size for path in case.rglob('*') if path.is_file()) > 128 * 1024**2:
                raise RuntimeError('capture/disk budget exhausted')
            request_path = control / 'request.json'
            if request_path.exists() and not record['targets']:
                request = json.loads(request_path.read_bytes())
                if request['uid'] != user.pw_uid or set(request['targets']) != set(ROLES):
                    raise ValueError('wrong business request owner/roles')
                for role, expected in request['targets'].items():
                    record['targets'][role] = checked_target(expected, user.pw_uid)
                    if mode == 'on':
                        output = bpf / role
                        output.mkdir()
                        trace_command = [str(BPF / 'trace_loader'), str(BPF / 'scheduler.bpf.o'), str(expected['pid']), '60', str(output / 'kernel.csv')]
                        log = (output / 'loader.log').open('xb')
                        logs.append(log)
                        children[role] = subprocess.Popen(trace_command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                        record['tracers'][role] = {'command': trace_command, 'identity': runtime_identity(children[role].pid)}
                attach_deadline = time.monotonic_ns() + 5000000000
                while mode == 'on':
                    if any(children[role].poll() is not None for role in ROLES):
                        raise RuntimeError('tracer exited before attach readiness')
                    links = {role: link_facts(children[role].pid) for role in ROLES}
                    if all(len(facts) == 5 for facts in links.values()):
                        record['attach_links'] = links
                        break
                    if time.monotonic_ns() > attach_deadline:
                        raise TimeoutError('five BPF links not observed for both targets')
                    time.sleep(.02)
                publish(control / 'ready.json', {'ready': True, 'mode': mode, 'requested_ns': request['requested_ns'],
                        'observed_attach_ready_ns': time.monotonic_ns(), 'targets': request['targets']})
            if now >= next_sample:
                if not (control / 'cutoff.json').exists():
                    for role, target in record['targets'].items():
                        record['thread_observations'][role].append(thread_observation(target))
                for role, child in children.items():
                    if role != 'business' and child.poll() is None:
                        resource.write(json.dumps(resource_sample(child.pid, 'tracer_' + role), sort_keys=True) + '\n')
                resource.write(json.dumps(resource_sample(os.getpid(), 'broker'), sort_keys=True) + '\n')
                resource.flush()
                next_sample = now + 1000000000
            if (control / 'cutoff.json').exists() and not trace_stopped:
                for role in ROLES:
                    if role in children:
                        record['exits'][role] = stop(children[role])
                trace_stopped = True
            elif record['targets'] and not trace_stopped and any(children[role].poll() is not None for role in ROLES if role in children):
                raise RuntimeError('tracer stopped before fixed business cutoff')
            time.sleep(.05)
        record['exits']['business'] = stop(children['business'])
    except Exception as error:
        record['errors'].append(type(error).__name__ + ': ' + str(error))
        record['traceback'] = traceback.format_exc()
    finally:
        for role, child in children.items():
            if role not in record['exits']:
                record['exits'][role] = stop(child)
        owners = control / 'owners.json'
        if owners.exists():
            for role, owner in json.loads(owners.read_bytes()).items():
                try:
                    fields = Path('/proc', str(owner['pid']), 'stat').read_text().rsplit(')', 1)[1].split()
                    if int(fields[19]) == owner['starttime'] and fields[0] != 'Z':
                        record['errors'].append('business owner remained after wait: ' + role)
                        os.kill(owner['pid'], signal.SIGTERM)
                except (FileNotFoundError, ProcessLookupError):
                    pass
        resource.flush()
        os.fsync(resource.fileno())
        resource.close()
        for log in logs:
            log.flush()
            os.fsync(log.fileno())
            log.close()
        record['flush_done_ns'] = time.monotonic_ns()
    if (case / 'scene/events.json').exists():
        try:
            business = json.loads((case / 'scene/events.json').read_bytes())
            if business['errors'] or any(row['exit'] or row.get('timeout') for row in business['exits'].values()):
                record['errors'].append('business peer error/nonzero/forced shutdown; preserve original logs')
            freeze(case / 'scene')
        except Exception as error:
            record['errors'].append('scene freeze: ' + str(error))
    else:
        record['errors'].append('missing completed business events')
    publish(case / 'broker.json', record)
    frozen_case(case)
    return not record['errors'] and all(row['exit'] == 0 and not row['forced'] for row in record['exits'].values())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=lambda value: Path(value).resolve())
    parser.add_argument('--user', default='zlm')
    args = parser.parse_args()
    user = pwd.getpwnam(args.user)
    if os.geteuid() != 0 or user.pw_uid == 0:
        parser.error('authorized root broker and nonroot business account required')
    if not args.output.is_relative_to(ROOT / 'reports'):
        parser.error('evidence output must remain inside ControlLink reports')
    if os.readlink('/proc/self/ns/net') == os.readlink('/proc/1/ns/net'):
        parser.error('refusing host network namespace; use run_business_bpf.bash')
    args.output.mkdir(parents=True, exist_ok=False)
    os.chown(args.output, user.pw_uid, user.pw_gid)
    publish(args.output / 'run-identity.json', {
        'head': subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
        'tree': subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD^{tree}'], text=True).strip(),
        'git_status': subprocess.check_output(['git', '-C', str(ROOT), 'status', '--porcelain=v1'], text=True),
        'kernel': list(os.uname()), 'vcpus': os.cpu_count(), 'meminfo': Path('/proc/meminfo').read_text(),
        'broker': runtime_identity(os.getpid()), 'tool_sha256': {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted((ROOT / 'tools/linux_depth').glob('*')) if path.is_file()},
        'bpf_sha256': {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in (BPF / 'trace_loader', BPF / 'scheduler.bpf.o')},
        'build_origin': 'reuse previously reviewed ELF from baseline plus source patch; no rebuild for current commit metadata',
        'references': {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in (
            ROOT / 'reports/linux-depth-20261021/remote/delivery/build-identity.json',
            ROOT / 'reports/linux-depth-20261021/remote/delivery/source-before-commit.json',
            ROOT / 'reports/linux-depth-20261021/remote/delivery/runtime-assembly.json')}})
    def cancelled(number, frame):
        raise InterruptedError('broker signal ' + str(number))
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, cancelled)
    for command in (['ip', 'link', 'set', 'lo', 'up'], ['ip', 'link', 'add', 'vcan0', 'type', 'vcan'], ['ip', 'link', 'set', 'vcan0', 'up']):
        subprocess.run(command, check=True)
    subprocess.run(['python3', str(ROOT / 'tools/linux_depth/bpf_preflight.py'), '--loader', str(BPF / 'trace_loader'),
                    '--output', str(args.output / 'preflight.json')], check=True)
    result = 1
    try:
        if all(run_case(args.output, mode, user, 87) for mode in ('off', 'on')):
            result = subprocess.run(['python3', str(ROOT / 'tools/linux_depth/analyze_business_bpf.py'),
                                     str(args.output), str(args.output / 'analysis')]).returncode
        publish(args.output / 'pair-result.json', {'exit': result, 'closed_monotonic_ns': time.monotonic_ns()})
    finally:
        frozen_case(args.output)
        publish(args.output / 'seal.json', {'capture_freeze_sha256': hashlib.sha256((args.output / 'capture-freeze.json').read_bytes()).hexdigest(),
                                          'scope': 'VM short observation only; not independent acceptance'})
    return result


if __name__ == '__main__':
    raise SystemExit(main())
