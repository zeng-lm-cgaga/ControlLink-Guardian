import copy
import csv
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from analyze_bpf import analyze_kernel, validate_controlled


class KernelAnalysisTest(unittest.TestCase):
    def test_synthetic_wait_and_syscall_overlap(self):
        raw = ("kind,tgid,tid,cpu,task_start_boottime_ns,at_ns,duration_ns,auxiliary_ns,value\n"
               "2,10,11,0,1,100,20,0,1\n1,10,11,0,1,150,0,0,0\n"
               "3,10,11,0,1,170,20,70,1\n5,10,11,0,1,180,80,202,0\n"
               "# footer end_ns=200 received=4 emitted=4 ring_full=0 map_full=0\n").encode()
        app = [{"phase": "work_lock", "pid": 10, "tid": 11, "source": "test", "gid": "00", "sequence": 1,
                "source_stamp_ns": 0, "begin_ns": 95, "end_ns": 190}]
        result = analyze_kernel(raw, b"loader_exit=0", app)
        self.assertTrue(result["population_complete"])
        self.assertEqual(result["application_overlaps"][0]["overlap_ns_by_kind"],
                         {"on_cpu": 5, "blocked_off_cpu": 70, "wakeup_wait": 20, "syscall_202": 80})
        self.assertNotIn("runnable_off_cpu_ms", result["metrics"])

    def test_unprivileged_attempt_not_pass(self):
        with self.assertRaisesRegex(ValueError, "incomplete"):
            analyze_kernel(b"# no footer\n", b"loader_exit=1")

    def test_dropped_events_explicit_partial(self):
        raw = b"kind,tgid,tid,cpu,task_start_boottime_ns,at_ns,duration_ns,auxiliary_ns,value\n# footer received=0 emitted=0 ring_full=5 map_full=0\n"
        self.assertFalse(analyze_kernel(raw, b"loader_exit=3")["population_complete"])

    def test_empty_accounting_is_not_valid_coverage(self):
        raw = b"# attached=5 start_monotonic_ns=10\nkind,tgid,tid,cpu,task_start_boottime_ns,at_ns,duration_ns,auxiliary_ns,value\n# footer received=0 emitted=0 ring_full=0 map_full=0\n"
        result = analyze_kernel(raw, b"loader_exit=0")
        self.assertTrue(result["accounting_complete"])
        self.assertFalse(result["population_complete"])
        self.assertFalse(validate_controlled(result, [], {"pid": 10, "starttime": 1, "clk_tck": 100, "threads": {}}, {"population_complete": True}))

    def controlled_fixture(self):
        raw = ("# monotonic_ns target_tgid=10 start_ticks=1 hz=100 target_tid=0\n# attached=5 start_monotonic_ns=1\n"
               "kind,tgid,tid,cpu,task_start_boottime_ns,at_ns,duration_ns,auxiliary_ns,value\n"
               "2,10,10,0,10000000,30,20,0,1\n1,10,10,0,10000000,40,0,0,0\n"
               "3,10,10,0,10000000,50,10,20,1\n4,10,10,0,10000000,60,0,0,230\n"
               "5,10,10,0,10000000,100,40,230,0\n5,10,10,0,10000000,200,80,202,0\n"
               "5,10,11,0,10000000,200,80,230,0\n"
               "# footer received=7 emitted=7 ring_full=0 map_full=0\n").encode()
        app = [{"phase": phase, "pid": 10, "tid": tid, "source": "controlled_workload", "gid": "00", "sequence": 1,
                "source_stamp_ns": 0, "begin_ns": begin, "end_ns": end}
               for phase, tid, begin, end in (("work_cpu", 10, 10, 30), ("work_sleep", 10, 60, 100),
                                            ("work_lock", 10, 120, 200), ("work_hold", 11, 120, 200))]
        target = {"pid": 10, "starttime": 1, "clk_tck": 100, "threads": {"10": {"starttime": 1}, "11": {"starttime": 1}}}
        return analyze_kernel(raw, b"loader_exit=0", app), app, target

    def test_controlled_semantics_accept_valid_synthetic_population(self):
        result, app, target = self.controlled_fixture()
        self.assertTrue(validate_controlled(result, app, target, {"population_complete": True}, iterations=1))

    def test_controlled_rejects_wrong_instance_missing_kind_and_no_overlap(self):
        result, app, target = self.controlled_fixture()
        for change in ("tgid", "tid", "starttime", "app_pid", "no_syscall", "no_overlap", "loss", "app_loss"):
            current, application, owner = copy.deepcopy((result, app, target))
            quality = {"population_complete": change != "app_loss"}
            if change == "tgid": owner["pid"] = 12
            if change == "tid": del owner["threads"]["11"]
            if change == "starttime": owner["threads"]["10"]["starttime"] = 2
            if change == "app_pid": application[0]["pid"] = 12
            if change == "no_syscall": del current["event_kinds"][4]
            if change == "no_overlap": current["application_overlaps"] = []
            if change == "loss": current["population_complete"] = False
            with self.subTest(change=change):
                self.assertFalse(validate_controlled(current, application, owner, quality, iterations=1))

    def test_controlled_cli_fails_closed_for_empty_kernel_capture(self):
        unused, app, target = self.controlled_fixture()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            raw = ("# monotonic_ns target_tgid=10 start_ticks=1 hz=100 target_tid=0\n# attached=5 start_monotonic_ns=1\n"
                   "kind,tgid,tid,cpu,task_start_boottime_ns,at_ns,duration_ns,auxiliary_ns,value\n"
                   "# footer received=0 emitted=0 ring_full=0 map_full=0\n")
            (root/'kernel.csv').write_text(raw)
            (root/'loader.log').write_text('loader_exit=0')
            (root/'target.json').write_text(json.dumps(target))
            rows = []
            for sequence in range(1, 101):
                for sample in app:
                    row = {**sample, 'sequence': sequence, 'source_hex': sample['source'].encode().hex(),
                           'scheduled_ns': 0, 'frame_id': 0, 'result': 0, 'value': 0}
                    del row['source']
                    rows.append(row)
            buffer = io.StringIO()
            buffer.write('# control_link_timing_v2 clock=CLOCK_MONOTONIC\n')
            writer = csv.DictWriter(buffer, fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)
            buffer.write('# drain queued=400 written=400\n')
            (root/'app.csv').write_text(buffer.getvalue())
            (root/'app.csv.status.json').write_text(json.dumps({'data_closed': True, 'queued': 400, 'written': 400,
                'io_errors': 0, 'full': 0, 'contended': 0, 'invalid_identity': 0, 'closed': 0}))
            result = subprocess.run([sys.executable, str(Path(__file__).with_name('analyze_bpf.py')), str(root/'kernel.csv'), str(root/'loader.log'),
                                     '--application', str(root/'app.csv'), '--target-identity', str(root/'target.json'),
                                     '--require-controlled-workload', '--output', str(root/'result.json')], capture_output=True, text=True)
            self.assertEqual(result.returncode, 1, result.stderr)
            report = json.loads((root/'result.json').read_bytes())
            self.assertTrue(report['accounting_complete'])
            self.assertFalse(report['controlled_acceptance']['passed'])
            self.assertIn('empty or lossy kernel population', report['controlled_acceptance']['errors'])


if __name__ == "__main__":
    unittest.main()
