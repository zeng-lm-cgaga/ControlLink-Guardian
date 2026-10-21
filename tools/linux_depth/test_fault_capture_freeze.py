import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest

from freeze_fault_capture import freeze


class FaultCaptureFreezeTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='control-link-freeze-test-')
        self.root = Path(self.temporary.name)
        self.capture = self.root / 'capture'
        self.capture.mkdir()
        self.output = self.root / 'manifest.json'
        self.pid, self.starttime = 999999999, 1
        self.status = {'closed': True, 'io_errors': 0, 'pid': self.pid,
                       'starttime': self.starttime, 'synthetic_test_fixture': True}
        self.write_status()
        (self.capture / 'continuous-1.jsonl').write_text('{"synthetic_test_event":1}\n')

    def tearDown(self):
        self.temporary.cleanup()

    def write_status(self):
        (self.capture / 'capture-status.json').write_text(json.dumps(self.status))

    def test_closed_partial_capture_has_exact_hashes_and_no_overwrite(self):
        freeze(self.capture, self.output, self.pid, self.starttime)
        result = json.loads(self.output.read_text())
        self.assertFalse(result['replay_complete'])
        self.assertEqual(len(result['artifacts']), 2)
        for artifact in result['artifacts']:
            self.assertEqual(artifact['sha256'], hashlib.sha256(Path(artifact['path']).read_bytes()).hexdigest())
        with self.assertRaises(FileExistsError):
            freeze(self.capture, self.output, self.pid, self.starttime)

    def test_live_owner_is_not_freezable(self):
        starttime = int(Path('/proc/self/stat').read_text().rsplit(')', 1)[1].split()[19])
        with self.assertRaisesRegex(ValueError, 'still present'):
            freeze(self.capture, self.output, os.getpid(), starttime)

    def test_writer_error_or_wrong_owner_is_not_success(self):
        for changes in ({'io_errors': 1}, {'pid': self.pid - 1}, {'closed': False}):
            original = dict(self.status)
            self.status.update(changes)
            self.write_status()
            with self.assertRaises(ValueError):
                freeze(self.capture, self.output, self.pid, self.starttime)
            self.status = original

    def test_unexpected_or_symlink_file_is_rejected(self):
        extra = self.capture / 'capture-status.pending'
        extra.write_text('synthetic incomplete marker')
        with self.assertRaises(ValueError):
            freeze(self.capture, self.output, self.pid, self.starttime)
        extra.unlink()
        extra.symlink_to(self.capture / 'capture-status.json')
        with self.assertRaises(ValueError):
            freeze(self.capture, self.output, self.pid, self.starttime)

    def test_manifest_is_outside_the_owned_budget(self):
        with self.assertRaises(ValueError):
            freeze(self.capture, self.capture / 'manifest.json', self.pid, self.starttime)


if __name__ == '__main__':
    unittest.main()
