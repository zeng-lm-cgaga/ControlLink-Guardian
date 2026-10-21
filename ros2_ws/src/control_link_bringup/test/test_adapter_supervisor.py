import importlib.util
from pathlib import Path
import threading
import unittest
from unittest.mock import patch

SOURCE = Path(__file__).parents[1] / 'control_link_bringup/adapter_supervisor.py'
SPEC = importlib.util.spec_from_file_location('adapter_supervisor', SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SupervisorTests(unittest.TestCase):
    def policy(self):
        return dict(max_restarts=2, restart_budget_ms=1000, backoff_initial_ms=1,
                    backoff_max_ms=2, graceful_stop_ms=1000, terminate_wait_ms=100)

    def test_normal_exit_is_not_restarted(self):
        rows = []
        owner = MODULE.AdapterSupervisor(['/bin/true'], self.policy(), rows.append)
        self.assertEqual(owner.run(), 0)
        self.assertEqual(sum(row['state'] == 'CHILD_REAPED' for row in rows), 1)
        self.assertIsNone(owner.child)

    def test_failure_budget_is_cumulative_and_reaped(self):
        rows = []
        owner = MODULE.AdapterSupervisor(['/bin/false'], self.policy(), rows.append)
        self.assertEqual(owner.run(), 75)
        self.assertEqual(sum(row['state'] == 'CHILD_REAPED' for row in rows), 3)
        self.assertTrue(any(row['state'] == 'EXHAUSTED_MANUAL_INTERVENTION_REQUIRED' for row in rows))

    def test_exec_failure_and_stop_interrupt_backoff(self):
        rows = []
        owner = MODULE.AdapterSupervisor(['/does-not-exist'], self.policy(), rows.append)
        def emit(row):
            rows.append(row)
            if row['state'] == 'BACKOFF':
                owner.request_stop()
        owner.emit = emit
        self.assertEqual(owner.run(), 0)
        self.assertEqual(sum(row['state'] == 'EXEC_FAILED' for row in rows), 1)

    def test_elapsed_budget_prevents_a_late_restart(self):
        rows = []
        policy = self.policy()
        policy.update(restart_budget_ms=1, backoff_initial_ms=10, backoff_max_ms=10)
        owner = MODULE.AdapterSupervisor(['/bin/false'], policy, rows.append)
        self.assertEqual(owner.run(), 75)
        self.assertEqual(sum(row['state'] == 'CHILD_STARTED_NOT_BUSINESS_READY' for row in rows), 1)
        self.assertIsNone(owner.child)

    def test_stop_waits_for_its_child_only(self):
        owner = MODULE.AdapterSupervisor(['/bin/sleep', '10'], self.policy(), lambda row: None)
        timer = threading.Timer(0.05, owner.request_stop)
        timer.start()
        try:
            self.assertEqual(owner.run(), 0)
            self.assertIsNone(owner.child)
        finally:
            timer.join()

    def test_backoff_oversleep_does_not_exec_replacement(self):
        rows, now = [], [10.0]
        def oversleep(seconds):
            now[0] += 2
            return False
        owner = MODULE.AdapterSupervisor(['/missing'], self.policy(), rows.append,
                                         clock=lambda: now[0], wait=oversleep)
        with patch.object(MODULE.subprocess, 'Popen', side_effect=OSError('fixture')) as launch:
            self.assertEqual(owner.run(), 75)
        self.assertEqual(launch.call_count, 1)
        self.assertEqual(rows[-2]['state'], 'EXHAUSTED_MANUAL_INTERVENTION_REQUIRED')

    def test_stop_has_priority_when_backoff_crosses_deadline(self):
        now = [10.0]
        def oversleep_and_stop(seconds):
            now[0] += 2
            owner.request_stop()
            return False
        owner = MODULE.AdapterSupervisor(['/missing'], self.policy(), lambda row: None,
                                         clock=lambda: now[0], wait=oversleep_and_stop)
        with patch.object(MODULE.subprocess, 'Popen', side_effect=OSError('fixture')) as launch:
            self.assertEqual(owner.run(), 0)
        self.assertEqual(launch.call_count, 1)

    def test_deadline_rechecked_immediately_before_replacement(self):
        readings = iter((10.0, 10.1, 12.0))
        owner = MODULE.AdapterSupervisor(['/missing'], self.policy(), lambda row: None,
                                         clock=lambda: next(readings), wait=lambda seconds: False)
        with patch.object(MODULE.subprocess, 'Popen', side_effect=OSError('fixture')) as launch:
            self.assertEqual(owner.run(), 75)
        self.assertEqual(launch.call_count, 1)


if __name__ == '__main__':
    unittest.main()
