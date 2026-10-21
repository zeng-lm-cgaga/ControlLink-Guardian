"""按 PID/starttime 固定实例；停止所有 owner 后才允许把日志称为冻结输入。"""
import json
import os
from pathlib import Path
import signal
import tempfile
import time


def process_instance(pid):
    fields = Path('/proc', str(pid), 'stat').read_text().rsplit(')', 1)[1].split()
    return {'pid': pid, 'starttime': int(fields[19]), 'state': fields[0]}


def instance_status(owner):
    try:
        actual = process_instance(owner['pid'])
    except (FileNotFoundError, ProcessLookupError):
        return {'exit_confirmed': True, 'state': 'gone'}
    except (OSError, ValueError, IndexError) as error:
        return {'exit_confirmed': False, 'state': 'unknown', 'error': str(error)}
    if actual['starttime'] != owner['starttime']:
        return {'exit_confirmed': True, 'state': 'identity_changed', 'actual': actual}
    return {'exit_confirmed': actual['state'] in ('Z', 'X'), 'state': actual['state']}


def write_owners(path, owners):
    """这是运行中可更新的 owner 登记，不是封存文件；replace 前先持久化临时文件。"""
    path = Path(path)
    descriptor, temporary = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(descriptor, 'w') as handle:
            json.dump(owners, handle, sort_keys=True)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def require_quiescent(owners):
    states = {role: instance_status(owner) for role, owner in owners.items()}
    if any(not row['exit_confirmed'] for row in states.values()):
        raise RuntimeError('cleanup not closed; final freeze forbidden: ' + json.dumps(states, sort_keys=True))
    return states


def stop_instances(owners, budgets=((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 2)), *, groups=False):
    """每轮先通知全部实例，再共用有界等待；pidfd 防止检查后 PID 复用误伤。"""
    results = {role: dict(owner, requested_ns=time.monotonic_ns(), signals=[], errors=[])
               for role, owner in owners.items()}
    descriptors = {}
    try:
        for role, owner in owners.items():
            status = instance_status(owner)
            if status['exit_confirmed']:
                continue
            if status['state'] == 'unknown':
                results[role]['errors'].append('unverified identity: ' + status['error'])
                continue
            try:
                descriptor = os.pidfd_open(owner['pid'])
                descriptors[role] = descriptor
                status = instance_status(owner)
                if status['exit_confirmed'] or status['state'] == 'unknown':
                    os.close(descriptors.pop(role))
            except (OSError, AttributeError) as error:
                results[role]['errors'].append('pin instance: ' + str(error))
        for number, seconds in budgets:
            pending = [role for role in descriptors if not instance_status(owners[role])['exit_confirmed']]
            if not pending:
                break
            for role in pending:
                owner = owners[role]
                try:
                    if groups and os.getpgid(owner['pid']) == owner['pid']:
                        os.killpg(owner['pid'], number)
                    else:
                        signal.pidfd_send_signal(descriptors[role], number)
                    results[role]['signals'].append({'signal': int(number), 'at_ns': time.monotonic_ns()})
                except (ProcessLookupError, FileNotFoundError):
                    pass
                except OSError as error:
                    results[role]['errors'].append('signal: ' + str(error))
            deadline = time.monotonic() + seconds
            while any(not instance_status(owners[role])['exit_confirmed'] for role in pending):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(.02, remaining))
        for role, owner in owners.items():
            results[role].update(instance_status(owner), waited_ns=time.monotonic_ns())
            results[role]['forced'] = any(row['signal'] != int(budgets[0][0]) for row in results[role]['signals'])
        return results
    finally:
        for descriptor in descriptors.values():
            os.close(descriptor)


def stop_children(children, budgets=((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 2))):
    """Popen owner 额外 wait/reap；单个失败不能跳过剩余 child 的清理。"""
    owners, results = {}, {}
    for role, child in children.items():
        code = child.poll()
        if code is not None:
            results[role] = {'pid': child.pid, 'exit': code, 'forced': False, 'exit_confirmed': True,
                             'wait_done_ns': time.monotonic_ns()}
        else:
            try:
                owners[role] = process_instance(child.pid)
            except (OSError, ValueError, IndexError) as error:
                results[role] = {'pid': child.pid, 'exit_confirmed': False, 'forced': True,
                                 'identity_error': str(error)}
    results.update(stop_instances(owners, budgets, groups=True))
    for role, child in children.items():
        try:
            results[role]['exit'] = child.wait(timeout=1)
            results[role]['exit_confirmed'] = True
        except Exception as error:
            results[role]['exit_confirmed'] = False
            results[role]['wait_error'] = type(error).__name__ + ': ' + str(error)
            results[role]['exit'] = child.poll()
        results[role]['wait_done_ns'] = time.monotonic_ns()
        results[role]['timeout'] = results[role].get('forced', False) or not results[role]['exit_confirmed']
    return results
