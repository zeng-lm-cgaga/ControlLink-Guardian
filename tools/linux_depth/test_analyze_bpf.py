import unittest
from analyze_bpf import analyze_kernel


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
                         {"blocked_off_cpu": 70, "wakeup_wait": 20, "syscall_202": 80})
        self.assertNotIn("runnable_off_cpu_ms", result["metrics"])

    def test_unprivileged_attempt_not_pass(self):
        with self.assertRaisesRegex(ValueError, "incomplete"):
            analyze_kernel(b"# no footer\n", b"loader_exit=1")

    def test_dropped_events_explicit_partial(self):
        raw = b"kind,tgid,tid,cpu,task_start_boottime_ns,at_ns,duration_ns,auxiliary_ns,value\n# footer received=0 emitted=0 ring_full=5 map_full=0\n"
        self.assertFalse(analyze_kernel(raw, b"loader_exit=3")["population_complete"])


if __name__ == "__main__":
    unittest.main()
