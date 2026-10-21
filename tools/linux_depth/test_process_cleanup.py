"""临时 writer/独立 session 的定向关闭检查，不启动 ROS/CAN/BPF。"""
import json
import os
from pathlib import Path
import pwd
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from business_trace import publish
from process_cleanup import instance_status, process_instance, require_quiescent, stop_children, stop_instances, write_owners
from run_business_bpf import frozen_case
import run_business_bpf


class ProcessCleanupTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.children = {}

    def tearDown(self):
        stop_children(self.children, ((signal.SIGTERM, .1), (signal.SIGKILL, .5)))
        self.directory.cleanup()

    def writer(self, mode):
        path = self.root / (mode + '.log')
        code = '''import os, signal, sys, time
mode, path = sys.argv[1:]
signal.signal(signal.SIGINT, signal.SIG_IGN)
def delayed(number, frame):
    time.sleep(.12)
    raise SystemExit(0)
signal.signal(signal.SIGTERM, delayed if mode == 'delay' else signal.SIG_IGN)
with open(path, 'wb', buffering=0) as stream:
    while True:
        stream.write(b'writer\\n')
        time.sleep(.01)
'''
        child = subprocess.Popen([sys.executable, '-c', code, mode, str(path)], start_new_session=True)
        self.children[mode] = child
        deadline = time.monotonic() + 3
        while not path.exists() or not path.stat().st_size:
            if child.poll() is not None or time.monotonic() > deadline:
                self.fail('writer did not become ready')
            time.sleep(.01)
        return child, path, process_instance(child.pid)

    def registry(self, owner):
        control = self.root / 'control'
        control.mkdir()
        write_owners(control / 'owners.json', {'writer': owner})

    def test_final_freeze_forbidden_until_writer_stops(self):
        child, path, owner = self.writer('ignore')
        self.registry(owner)
        with self.assertRaisesRegex(RuntimeError, 'freeze forbidden'):
            frozen_case(self.root)
        self.assertFalse((self.root / 'capture-freeze.json').exists())
        result = stop_instances({'writer': owner}, ((signal.SIGTERM, .03), (signal.SIGKILL, .5)))
        self.assertTrue(result['writer']['exit_confirmed'])
        self.assertTrue(result['writer']['forced'])
        self.assertEqual(child.wait(timeout=1), -signal.SIGKILL)
        size = path.stat().st_size
        frozen_case(self.root)
        time.sleep(.03)
        self.assertEqual(path.stat().st_size, size)
        self.assertTrue((self.root / 'capture-freeze.json').exists())

    def test_delayed_term_waited_before_freeze(self):
        child, path, owner = self.writer('delay')
        self.registry(owner)
        began = time.monotonic()
        result = stop_instances({'writer': owner}, ((signal.SIGTERM, .5), (signal.SIGKILL, .5)))
        self.assertGreaterEqual(time.monotonic() - began, .10)
        self.assertEqual(child.wait(timeout=1), 0)
        self.assertEqual(len(result['writer']['signals']), 1)
        require_quiescent({'writer': owner})
        frozen_case(self.root)

    def test_changed_identity_never_signalled(self):
        child, path, owner = self.writer('ignore')
        owner['starttime'] += 1
        result = stop_instances({'writer': owner}, ((signal.SIGTERM, .01),))
        self.assertEqual(result['writer']['state'], 'identity_changed')
        self.assertEqual(result['writer']['signals'], [])
        self.assertIsNone(child.poll())

    def test_disappeared_instance_is_closed(self):
        child = subprocess.Popen(['/bin/true'])
        self.children['short'] = child
        owner = process_instance(child.pid)
        child.wait(timeout=1)
        result = stop_instances({'short': owner})
        self.assertTrue(result['short']['exit_confirmed'])
        self.assertEqual(result['short']['signals'], [])

    def test_unreadable_identity_does_not_signal_or_allow_freeze(self):
        owner = {'pid': os.getpid(), 'starttime': 1}
        with patch('process_cleanup.process_instance', side_effect=PermissionError('fixture')):
            result = stop_instances({'unknown': owner})
            self.assertFalse(result['unknown']['exit_confirmed'])
            self.assertEqual(result['unknown']['signals'], [])
            with self.assertRaises(RuntimeError):
                require_quiescent({'unknown': owner})

    def test_wait_failure_does_not_skip_other_owner(self):
        first, unused, first_owner = self.writer('ignore')
        second, unused, second_owner = self.writer('delay')
        with patch.object(first, 'wait', side_effect=subprocess.TimeoutExpired('fixture', 1)):
            result = stop_children({'first': first, 'second': second},
                                   ((signal.SIGTERM, .2), (signal.SIGKILL, .5)))
        self.assertFalse(result['first']['exit_confirmed'])
        self.assertEqual(result['second']['exit'], 0)
        self.assertTrue(result['second']['exit_confirmed'])
        first.wait(timeout=1)

    def test_incomplete_cleanup_blocks_outer_freeze_even_without_live_pid(self):
        case = self.root / 'off'
        case.mkdir()
        publish(case / 'broker.json', {'cleanup_closed': False})
        with self.assertRaisesRegex(RuntimeError, 'cleanup incomplete'):
            frozen_case(self.root)
        self.assertFalse((self.root / 'capture-freeze.json').exists())

    def test_driver_failure_leaves_separate_session_writer_to_close(self):
        child, path, owner = self.writer('ignore')
        self.registry(owner)
        driver = subprocess.Popen([sys.executable, '-c', 'raise RuntimeError("driver fixture")'],
                                  stderr=subprocess.DEVNULL, start_new_session=True)
        self.children['driver'] = driver
        self.assertNotEqual(driver.wait(timeout=1), 0)
        self.assertFalse(instance_status(owner)['exit_confirmed'])
        with self.assertRaises(RuntimeError):
            frozen_case(self.root)
        stop_instances({'writer': owner}, ((signal.SIGTERM, .03), (signal.SIGKILL, .5)))
        child.wait(timeout=1)
        frozen_case(self.root)

    def test_broker_closes_driver_orphan_before_immutable_index(self):
        actual_popen = subprocess.Popen
        writer = "import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(30)"
        driver = '''import json, pathlib, subprocess, sys, time
control = pathlib.Path(sys.argv[1])
child = subprocess.Popen([sys.executable, '-c', sys.argv[2]], start_new_session=True)
time.sleep(.05)
fields = pathlib.Path('/proc', str(child.pid), 'stat').read_text().rsplit(')', 1)[1].split()
(control/'owners.json').write_text(json.dumps({'orphan': {'pid':child.pid,'starttime':int(fields[19])}}))
(control/'owners-ready.json').write_text(json.dumps({'registered_roles':['orphan']}))
raise SystemExit(7)
'''
        def spawn(command, **options):
            child = actual_popen([sys.executable, '-c', driver, str(self.root / 'off/control'), writer], **options)
            self.children['driver'] = child
            return child
        actual_stop = stop_instances
        def bounded_stop(owners):
            return actual_stop(owners, ((signal.SIGTERM, .03), (signal.SIGKILL, .5)))
        with patch.object(run_business_bpf.subprocess, 'Popen', side_effect=spawn), \
             patch.object(run_business_bpf, 'stop_instances', side_effect=bounded_stop):
            self.assertFalse(run_business_bpf.run_case(self.root, 'off', pwd.getpwuid(os.getuid()), 95, self.root))
        record = json.loads((self.root / 'off/broker.json').read_bytes())
        self.assertTrue(record['cleanup_closed'])
        self.assertTrue(record['owner_cleanup']['orphan']['exit_confirmed'])
        self.assertTrue(record['owner_cleanup']['orphan']['forced'])
        self.assertGreaterEqual(record['flush_done_ns'], record['owner_cleanup']['orphan']['waited_ns'])
        self.assertTrue((self.root / 'off/capture-freeze.json').exists())
        self.assertTrue(record['errors'])

    def test_broker_incomplete_startup_registration_is_not_frozen(self):
        actual_popen = subprocess.Popen
        def spawn(command, **options):
            child = actual_popen([sys.executable, '-c',
                'from pathlib import Path; import sys; Path(sys.argv[1]).write_text("{}"); raise SystemExit(9)',
                str(self.root / 'off/control/owners.json')], **options)
            self.children['driver'] = child
            return child
        with patch.object(run_business_bpf.subprocess, 'Popen', side_effect=spawn):
            self.assertFalse(run_business_bpf.run_case(self.root, 'off', pwd.getpwuid(os.getuid()), 95, self.root))
        record = json.loads((self.root / 'off/broker.json').read_bytes())
        self.assertFalse(record['cleanup_closed'])
        self.assertNotIn('flush_done_ns', record)
        self.assertFalse((self.root / 'off/capture-freeze.json').exists())


if __name__ == '__main__':
    unittest.main()
