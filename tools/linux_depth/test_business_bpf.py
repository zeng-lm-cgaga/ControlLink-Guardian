"""仅最小实现检查；数据均为合成，不代表真实 ROS/BPF 验收。"""
import copy
import json
from pathlib import Path
import tempfile
import unittest

from analyze_bpf import analyze_kernel
from analyze_business_bpf import compare_cases, correlate, resource_metrics
from business_trace import TraceGate, publish


class BusinessImplementationTest(unittest.TestCase):
    def test_gate_waits_for_matching_reply_and_health(self):
        with tempfile.TemporaryDirectory() as name:
            gate = TraceGate(name)
            targets = {'gateway': {'pid': 10, 'starttime': 99}}
            self.assertFalse(gate.ready(targets, False, 100))
            self.assertFalse((Path(name) / 'request.json').exists())
            self.assertFalse(gate.ready(targets, True, 101))
            publish(Path(name) / 'ready.json', {'requested_ns': 101, 'ready': True, 'mode': 'on', 'targets': targets})
            self.assertFalse(gate.ready(targets, False, 102))
            self.assertTrue(gate.ready(targets, True, 103))

    def test_gate_rejects_wrong_instance_and_times_out(self):
        with tempfile.TemporaryDirectory() as name:
            gate = TraceGate(name)
            gate.ready({'gateway': {'pid': 10}}, True, 100)
            with self.assertRaises(TimeoutError):
                gate.ready({'gateway': {'pid': 10}}, True, 20000000101)
            publish(Path(name) / 'ready.json', {'requested_ns': 100, 'ready': True, 'mode': 'on', 'targets': {'gateway': {'pid': 11}}})
            gate.reply = None
            with self.assertRaises(ValueError):
                gate.ready({'gateway': {'pid': 10}}, True, 102)

    def test_exclusive_publish_never_overwrites_evidence(self):
        with tempfile.TemporaryDirectory() as name:
            path = Path(name) / 'proof.json'
            publish(path, {'first': True})
            with self.assertRaises(FileExistsError):
                publish(path, {'first': False})
            self.assertEqual(json.loads(path.read_bytes()), {'first': True})

    @staticmethod
    def synthetic():
        content = (b'# monotonic_ns target_tgid=10 start_ticks=99 hz=100 target_tid=0\n'
                   b'# attached=5 start_monotonic_ns=40\n'
                   b'kind,tgid,tid,cpu,task_start_boottime_ns,at_ns,duration_ns,auxiliary_ns,value\n'
                   b'2,10,11,0,10000000,100,30,0,1\n'
                   b'1,10,11,0,10000000,120,0,0,0\n'
                   b'3,10,11,0,10000000,130,10,30,1\n'
                   b'4,10,11,0,10000000,140,0,0,1\n'
                   b'5,10,11,0,10000000,160,20,1,16\n'
                   b'# footer end_ns=300 received=5 emitted=5 ring_full=0 map_full=0\n')
        app = [{'phase': 'can_write', 'pid': 10, 'tid': 11, 'source': 'planning', 'gid': 'ab' * 24,
                'sequence': 42, 'source_stamp_ns': 12345, 'frame_id': 500, 'result': 16, 'begin_ns': 141, 'end_ns': 159}]
        target = {'pid': 10, 'starttime': 99, 'clk_tck': 100, 'threads': {'11': {'starttime': 1}}}
        return content, app, target, {'start_ns': 50, 'end_ns': 250}

    def test_exact_task_and_application_interval_evidence(self):
        content, app, target, window = self.synthetic()
        result = correlate(content, b'loader_exit=0', app, target, [], window)
        self.assertTrue(result['accepted'], result['errors'])
        evidence = result['evidence'][0]
        self.assertEqual(evidence['sequence'], 42)
        self.assertEqual(evidence['gid'], 'ab' * 24)
        self.assertEqual(evidence['kernel_intersections'][0]['intersection_ns'], 18)
        self.assertEqual(evidence['kernel_intersections'][0]['kernel_event_index_zero_based'], 4)
        self.assertNotIn('kernel_intervals', analyze_kernel(content, b'loader_exit=0'))

    def test_identity_window_and_missing_kernel_rejected(self):
        content, app, target, window = self.synthetic()
        for change in ('tgid', 'tid_instance', 'window', 'empty'):
            with self.subTest(change=change):
                modified, owner, limits = content, copy.deepcopy(target), copy.deepcopy(window)
                if change == 'tgid':
                    owner['pid'] = 12
                if change == 'tid_instance':
                    owner['threads']['11']['starttime'] = 2
                if change == 'window':
                    limits['start_ns'] = 30
                if change == 'empty':
                    modified = b'\n'.join(line for line in content.splitlines() if not line[:1].isdigit()).replace(b'received=5 emitted=5', b'received=0 emitted=0')
                self.assertFalse(correlate(modified, b'loader_exit=0', app, owner, [], limits)['accepted'])

    def test_logical_mailbox_residence_not_worker_cpu(self):
        content, app, target, window = self.synthetic()
        app[0]['phase'] = 'mailbox_consume'
        result = correlate(content, b'loader_exit=0', app, target, [], window)
        self.assertFalse(result['accepted'])
        self.assertEqual(result['evidence'], [])

    def test_cpu_is_interval_one_core_and_rss_is_resident(self):
        rows = [{'role': 'tracer_gateway', 'pid': 20, 'starttime': 200, 'monotonic_ns': 1000000000,
                 'cpu_ticks': 10, 'rss_kib': 100},
                {'role': 'tracer_gateway', 'pid': 20, 'starttime': 200, 'monotonic_ns': 3000000000,
                 'cpu_ticks': 110, 'rss_kib': 120}]
        result = resource_metrics(rows, 0, 4000000000, 100)['tracer_gateway']
        self.assertEqual(result['cpu_mean_one_core_pct'], 50)
        self.assertEqual(result['rss_kib']['mean'], 110)
        rows[1]['starttime'] = 201
        with self.assertRaisesRegex(ValueError, 'instance'):
            resource_metrics(rows, 0, 4000000000, 100)

    @staticmethod
    def pair():
        roles = ('gateway', 'can_adapter', 'planning_ingress', 'simulator', 'idle_teleop', 'fixture_observer')
        events = {key: {} for key in ('config_sha256', 'kernel', 'environment')}
        events.update(driver_sha256='same', clk_tck=100, vcpus=8,
                      preflight={'gateway': {'sha256': 'same', 'ldd_r': 'address1'}},
                      identities={role: {'mapped_sha256': {'/lib/example.so': 'same'}} for role in roles},
                      arguments={'timing': True, 'cpu_load': False, 'pause_source': False},
                      window={'duration_s': 20, 'warmup_s': 10})
        case = {'events': events, 'summary': {'latencies_ms': {}, 'resource': {
            role: {'samples': 2, 'cpu_mean_one_core_pct': 2, 'rss_kib': {'mean': 100}} for role in roles}}}
        return {'off': case, 'on': copy.deepcopy(case)}

    def test_pair_checks_loaded_bytes_not_ldd_aslr_addresses(self):
        cases = self.pair()
        cases['on']['events']['preflight']['gateway']['ldd_r'] = 'address2'
        self.assertEqual(compare_cases(cases)['business_process_deltas']['gateway']['cpu_percentage_point_delta'], 0)
        cases['on']['events']['identities']['gateway']['mapped_sha256']['/lib/example.so'] = 'changed'
        with self.assertRaisesRegex(ValueError, 'loaded code'):
            compare_cases(cases)

    def test_pair_excludes_startup_only_activator(self):
        cases = self.pair()
        for case in cases.values():
            case['summary']['resource']['activator'] = {'samples': 0, 'cpu_mean_one_core_pct': None}
        result = compare_cases(cases)
        self.assertNotIn('activator', result['business_process_deltas'])
        self.assertEqual(result['excluded_resource_roles']['activator']['off_samples'], 0)

    def test_pair_requires_all_business_resource_intervals(self):
        for change in ('missing', 'one_sample', 'no_cpu'):
            with self.subTest(change=change):
                cases = self.pair()
                rows = cases['on']['summary']['resource']
                if change == 'missing':
                    del rows['gateway']
                elif change == 'one_sample':
                    rows['gateway']['samples'] = 1
                else:
                    rows['gateway']['cpu_mean_one_core_pct'] = None
                with self.assertRaisesRegex(ValueError, 'insufficient.*gateway'):
                    compare_cases(cases)


if __name__ == '__main__':
    unittest.main()
