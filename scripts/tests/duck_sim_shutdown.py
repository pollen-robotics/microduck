#!/usr/bin/env python3
"""Exercise duck-sim down against temporary child processes, without a simulator."""

import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "duck-sim"


class ShutdownIdentity(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="duck-sim-stop-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bin = self.root / "bin"
        self.bin.mkdir()
        for name in ("dirname", "tr", "rm", "grep", "cut", "sleep", "ps"):
            (self.bin / name).symlink_to(shutil.which(name))

        # No system services or body server belong to this fixture. An isolated PATH
        # keeps down away from the host's services and makes its port probe offline.
        for name in ("sudo", "python3"):
            stub = self.bin / name
            stub.write_text("#!/bin/sh\nexit 1\n")
            stub.chmod(0o755)

    def stop(self, state_name, process_state_name, should_stop):
        state = self.root / state_name
        state.mkdir()
        socket = self.root / process_state_name / "duck-a.sock"
        child = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(60)", str(socket)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        # Reap promptly: kill -0 must not mistake an exited child for a live one.
        reaper = threading.Thread(target=child.wait)
        reaper.start()
        try:
            (state / "duck-a.pid").write_text(str(child.pid))
            result = subprocess.run(
                ["/bin/sh", str(SCRIPT), "down"],
                env={**os.environ, "PATH": str(self.bin), "DUCK_SIM_STATE": str(state)},
                text=True,
                capture_output=True,
                timeout=10,
            )
            output = result.stdout + result.stderr
            self.assertEqual(result.returncode, 0, output)
            self.assertEqual(child.poll() is not None, should_stop, output)
        finally:
            if child.poll() is None:
                child.kill()
            reaper.join(timeout=5)

    def test_stops_own_process(self):
        self.stop("duck-normal", "duck-normal", should_stop=True)

    def test_stops_own_process_with_brackets_in_state_path(self):
        # A valid directory name must not be interpreted as a regex character class.
        self.stop("duck[1]", "duck[1]", should_stop=True)

    def test_stops_own_process_with_unmatched_bracket_in_state_path(self):
        self.stop("duck[", "duck[", should_stop=True)

    def test_preserves_another_state_directory_process(self):
        self.stop("duck-one", "duck-two", should_stop=False)

    def test_preserves_near_match_after_pid_reuse(self):
        # A stale pidfile can point to another instance. A dot in this instance's
        # state path must not match an arbitrary character in the other's path.
        self.stop("duck.one", "duckXone", should_stop=False)


if __name__ == "__main__":
    unittest.main(verbosity=2)
