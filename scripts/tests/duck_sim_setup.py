#!/usr/bin/env python3
"""CPU-only entry-point regressions: python3 scripts/tests/duck_sim_setup.py [duck-sim].

Each case copies the script into a disposable checkout, with its own HOME/state,
restricted PATH and loopback port. Cargo stops at a sentinel, so no daemons,
container commands, downloads or real robot connections can be launched.
"""
import contextlib
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import socket
import socketserver
import subprocess
import sys
import tempfile
import threading
import time
import unittest

SCRIPT = Path(sys.argv.pop(1) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / 'duck-sim').resolve()


class Body(socketserver.ThreadingTCPServer):
    daemon_threads = True
    allow_reuse_address = True


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        try:
            for raw in self.rfile:
                req = json.loads(raw)
                if self.server.bad:
                    self.wfile.write(self.server.bad)
                elif req['op'] == 'hello':
                    self.wfile.write(b'{"protocol":1,"joints":15}\n')
                else:
                    self.wfile.write(json.dumps({'trunk_z': self.server.height, 'imu': {'gravity': [0, 0, self.server.gravity]}}).encode() + b'\n')
                self.wfile.flush()
        except (OSError, ValueError):
            pass


class Setup(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='duck setup ')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.repo = self.root / 'checkout with spaces'
        (self.repo / 'scripts').mkdir(parents=True)
        self.script = self.repo / 'scripts/duck-sim'
        shutil.copyfile(SCRIPT, self.script)
        self.script.chmod(0o755)
        (self.repo / 'Cargo.toml').write_text('[workspace.package]\nrust-version = "1.89"\n')
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        for cmd in ['dirname', 'sed', 'head', 'awk', 'sort', 'find', 'mkdir', 'rm', 'cut', 'tr', 'grep', 'tail', 'sleep', 'cat', 'ps']:
            (self.bin / cmd).symlink_to(shutil.which(cmd))
        (self.bin / 'python3').symlink_to(sys.executable)
        self.tool('git', 'echo fixture-branch')
        self.tool('cargo', 'echo CARGO_REACHED; exit 73')
        self.tool('sudo', 'echo forbidden-sudo >> "$TEST_SUDO_LOG"; exit 1')
        self.tool('mountpoint', 'exit 1')
        self.tool('systemctl', 'exit 1')
        self.rust('rustc 1.89.0 (abc 2025-08-07)')
        self.state = self.root / 'state with spaces'
        self.state.mkdir()
        self.fixture_home = self.root / 'home with spaces'
        self.fixture_home.mkdir()
        # Bind without listening, retaining the reservation until this case finishes.
        self.reserved = socket.socket()
        self.reserved.bind(('127.0.0.1', 0))
        self.addCleanup(self.reserved.close)
        self.env = {'PATH': str(self.bin), 'HOME': str(self.fixture_home), 'LC_ALL': 'C',
                    'DUCK_SIM_STATE': str(self.state), 'DUCK_SIM_PORT': str(self.reserved.getsockname()[1]),
                    'DUCK_SIM_VIEWER': '0', 'TEST_SUDO_LOG': str(self.root / 'sudo.log')}
        target = self.repo / 'target/debug'
        target.mkdir(parents=True)
        self.ctl = target / 'robotctl'
        self.write_tool(self.ctl, 'printf "robot healthy\\nloop 50 Hz\\nthird\\nfourth\\nfifth\\n"; exit "${TEST_HEALTH_EXIT:-0}"')
        print('FIXTURE ' + json.dumps({'script': str(SCRIPT), 'cwd': str(self.root), 'env': self.env}), flush=True)

    def write_tool(self, path, body):
        path.write_text('#!/bin/sh\n' + body + '\n')
        path.chmod(0o755)

    def tool(self, name, body):
        self.write_tool(self.bin / name, body)

    def rust(self, output, code=0):
        self.tool('rustc', 'printf "%s\\n" ' + shlex.quote(output) + f'; exit {code}')

    def rl(self, path=None, venv=True, ort=True):
        path = path or self.root / 'microduck_rl'
        path.mkdir(parents=True, exist_ok=True)
        if venv:
            (path / '.venv/bin').mkdir(parents=True)
            self.write_tool(path / '.venv/bin/python', 'exit 99')
        if ort:
            lib = path / '.venv/lib/python3.12/site-packages/onnxruntime/capi'
            lib.mkdir(parents=True)
            (lib / 'libonnxruntime.so.1').touch()
        return path

    def run_script(self, *args, code=None, contains=None, absent=None):
        p = subprocess.Popen(['/bin/sh', str(self.script), *args], env=self.env, cwd=self.root,
                             text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             start_new_session=True)
        try:
            p.stdout_text, p.stderr_text = p.communicate(timeout=6)
        except subprocess.TimeoutExpired:
            # The old simlog leaves tail following forever. Kill only this fixture's group.
            os.killpg(p.pid, signal.SIGKILL)
            out, err = p.communicate(timeout=3)
            print(f'COMMAND {args!r} TIMEOUT (group cleaned up)\n{out}{err}', flush=True)
            self.fail(f'{args!r} did not return within six seconds')
        p = subprocess.CompletedProcess(p.args, p.returncode, p.stdout_text, p.stderr_text)
        print(f'COMMAND {shlex.join(p.args)} EXIT {p.returncode}\nSTDOUT:\n{p.stdout}STDERR:\n{p.stderr}', flush=True)
        if code is not None:
            self.assertEqual(p.returncode, code, p.stdout + p.stderr)
        if contains is not None:
            self.assertIn(contains, p.stdout + p.stderr)
        if absent is not None:
            self.assertNotIn(absent, p.stdout + p.stderr)
        return p

    @contextlib.contextmanager
    def body(self, height=.116, gravity=-1., bad=False):
        with Body(('127.0.0.1', 0), Handler) as server:
            server.height, server.gravity, server.bad = height, gravity, bad
            self.env['DUCK_SIM_PORT'] = str(server.server_address[1])
            thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .02})
            thread.start()
            try:
                yield
            finally:
                server.shutdown()
                thread.join()

    def test_no_checkout(self):
        self.run_script('up', code=1, contains='clone it beside this one', absent='Traceback')

    def test_sibling_discovery(self):
        self.rl()
        self.run_script('up', code=73, contains='CARGO_REACHED')

    def test_empty_override_uses_sibling(self):
        self.env['DUCK_SIM_RL'] = ''
        self.rl()
        self.run_script('up', code=73, contains='CARGO_REACHED')

    def test_historical_fallback(self):
        self.rl(self.fixture_home / 'Pollen/microduck_rl')
        self.run_script('up', code=73, contains='CARGO_REACHED')

    def test_sibling_wins_over_fallback(self):
        self.rl(venv=False, ort=False)
        self.rl(self.fixture_home / 'Pollen/microduck_rl')
        self.run_script('up', code=1, contains='has no .venv', absent='CARGO_REACHED')

    def test_override_wins_over_sibling(self):
        self.rl()
        self.env['DUCK_SIM_RL'] = str(self.root / 'custom missing')
        self.run_script('up', code=1, contains='custom missing', absent='CARGO_REACHED')

    def test_override_spaces(self):
        self.env['DUCK_SIM_RL'] = str(self.rl(self.root / 'custom simulator'))
        self.run_script('up', code=73, contains='CARGO_REACHED')

    def test_relative_override(self):
        self.rl(self.root / 'relative simulator')
        self.env['DUCK_SIM_RL'] = 'relative simulator'
        self.run_script('up', code=73, contains='CARGO_REACHED')

    def test_override_file(self):
        p = self.root / 'file'
        p.touch()
        self.env['DUCK_SIM_RL'] = str(p)
        self.run_script('up', code=1, contains='no simulator')

    def test_missing_venv(self):
        self.env['DUCK_SIM_RL'] = str(self.rl(venv=False, ort=False))
        self.run_script('up', code=1, contains='uv sync')

    def test_missing_onnxruntime(self):
        self.env['DUCK_SIM_RL'] = str(self.rl(ort=False))
        self.run_script('up', code=1, contains='no onnxruntime')

    def test_nonexecutable_venv(self):
        path = self.rl()
        (path / '.venv/bin/python').chmod(0o644)
        self.env['DUCK_SIM_RL'] = str(path)
        self.run_script('up', code=1, contains='uv sync')

    def test_macos_runtime_filename(self):
        path = self.rl()
        lib = path / '.venv/lib/python3.12/site-packages/onnxruntime/capi'
        (lib / 'libonnxruntime.so.1').rename(lib / 'libonnxruntime.1.dylib')
        self.env['DUCK_SIM_RL'] = str(path)
        self.run_script('up', code=73, contains='CARGO_REACHED')

    def test_broken_sibling_symlink_uses_fallback(self):
        (self.root / 'microduck_rl').symlink_to(self.root / 'absent', target_is_directory=True)
        self.rl(self.fixture_home / 'Pollen/microduck_rl')
        self.run_script('up', code=73, contains='CARGO_REACHED')

    def test_rust_versions(self):
        self.env['DUCK_SIM_RL'] = str(self.rl())
        for version, good in [('1.85.1', False), ('1.88.9', False), ('1.89.0', True),
                              ('1.89.1', True), ('1.90.0', True), ('1.100.0', True),
                              ('2.0.0', True), ('1.90.0-nightly', True), ('1.90.0-beta.2', True)]:
            with self.subTest(version=version):
                self.rust(f'rustc {version} (abc 2025-08-07)')
                self.run_script('up', code=73 if good else 1,
                                contains='CARGO_REACHED' if good else 'this workspace needs Rust 1.89',
                                absent=None if good else 'CARGO_REACHED')

    def test_patch_msrv(self):
        self.env['DUCK_SIM_RL'] = str(self.rl())
        (self.repo / 'Cargo.toml').write_text('[workspace.package]\nrust-version = "1.89.1"\n')
        self.run_script('up', code=1, contains='needs Rust 1.89.1', absent='CARGO_REACHED')

    def test_rust_error_does_not_stop_existing_state(self):
        self.env['DUCK_SIM_RL'] = str(self.rl())
        self.rust('rustc 1.85.1 (old compiler)')
        pidfile = self.state / 'duck-a.pid'
        pidfile.write_text('1')  # Never signals PID 1, even in the original implementation.
        self.run_script('up', code=1, contains='needs Rust 1.89', absent='CARGO_REACHED')
        self.assertTrue(pidfile.exists(), 'Rust preflight removed existing state')
        self.assertFalse((self.root / 'sudo.log').exists(), 'Rust preflight reached teardown')

    def test_rust_checked_from_repo(self):
        self.env['DUCK_SIM_RL'] = str(self.rl())
        self.env['EXPECTED_REPO'] = str(self.repo)
        self.tool('rustc', '[ "$PWD" = "$EXPECTED_REPO" ] || exit 44; echo "rustc 1.89.0 (abc)"')
        self.run_script('up', code=73, contains='CARGO_REACHED')

    def test_indented_msrv(self):
        self.env['DUCK_SIM_RL'] = str(self.rl())
        (self.repo / 'Cargo.toml').write_text('[workspace.package]\n  rust-version = "1.90"\n')
        self.run_script('up', code=1, contains='needs Rust 1.90', absent='CARGO_REACHED')

    def test_no_msrv_keeps_build_working(self):
        self.env['DUCK_SIM_RL'] = str(self.rl())
        (self.repo / 'Cargo.toml').write_text('[workspace.package]\n')
        self.run_script('up', code=73, contains='CARGO_REACHED')

    def test_missing_rustc(self):
        self.env['DUCK_SIM_RL'] = str(self.rl())
        (self.bin / 'rustc').unlink()
        self.run_script('up', code=1, contains='Install Rust', absent='CARGO_REACHED')

    def test_failed_rustc_keeps_diagnostic(self):
        self.env['DUCK_SIM_RL'] = str(self.rl())
        self.tool('rustc', 'echo "error: toolchain unavailable" >&2; exit 42')
        self.run_script('up', code=1, contains='toolchain unavailable', absent='CARGO_REACHED')

    def test_failed_rustc_with_version_output(self):
        self.env['DUCK_SIM_RL'] = str(self.rl())
        self.rust('rustc 1.90.0 (broken wrapper)', 42)
        self.run_script('up', code=1, contains='rustc --version failed', absent='CARGO_REACHED')

    def test_malformed_rust_version(self):
        self.env['DUCK_SIM_RL'] = str(self.rl())
        self.rust('rustc banana')
        self.run_script('up', code=1, contains='cannot parse rustc version', absent='CARGO_REACHED')

    def test_down_status_fresh_checkout(self):
        self.ctl.unlink()
        self.run_script('status', code=1, contains='the sim is not running', absent='not found')

    def test_down_status(self):
        self.run_script('status', code=1, contains='the sim is not running', absent='Traceback')

    def test_live_status(self):
        with self.body():
            self.run_script('status', code=0, contains='standing (trunk 0.116', absent='fifth')

    def test_status_unhealthy_ctl(self):
        self.env['TEST_HEALTH_EXIT'] = '7'
        with self.body():
            self.run_script('status', code=7)

    def test_status_bad_body_protocol(self):
        for payload in [b'not-json\n', b'null\n', b'{}\n', b'{"imu": {"gravity": []}}\n']:
            with self.subTest(payload=payload), self.body(bad=payload):
                self.run_script('status', code=1, contains='cannot read the simulated body', absent='Traceback')

    def test_seated_not_standing(self):
        with self.body(height=.07):
            self.run_script('status', code=1, contains='did not stand up')

    def test_fallen_not_standing(self):
        with self.body(gravity=0.):
            self.run_script('status', code=1, contains='did not stand up')

    def test_missing_log(self):
        self.run_script('simlog', code=1, contains='the sim is not running', absent='tail:')

    def test_stopped_log_last_50(self):
        (self.state / 'body.log').write_text(''.join(f'line-{i:03}\n' for i in range(80)))
        p = self.run_script('simlog', code=1, contains='line-079', absent='line-029')
        self.assertEqual(sum(x.startswith('line-') for x in p.stdout.splitlines()), 50)
        self.assertIn('the sim is not running', p.stdout)

    def test_live_log_follows(self):
        log = self.state / 'body.log'
        log.write_text('initial-line\n')
        with self.body():
            p = subprocess.Popen(['/bin/sh', str(self.script), 'simlog'], env=self.env,
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, start_new_session=True)
            try:
                time.sleep(.3)
                self.assertIsNone(p.poll())
                with log.open('a') as out:
                    out.write('appended-live-line\n')
                time.sleep(1.2)
                self.assertIsNone(p.poll())
            finally:
                os.killpg(p.pid, signal.SIGTERM)
                out, err = p.communicate(timeout=3)
                print(f'LIVE FOLLOW (terminated test-owned group) EXIT {p.returncode}\n{out}{err}', flush=True)
            self.assertIn('appended-live-line', out)
            self.assertNotIn('not running', out)

    def test_empty_down_idempotent(self):
        self.run_script('down', code=0, contains='down')
        self.run_script('down', code=0, contains='down')
        self.run_script('status', code=1, contains='not running')

    def test_stale_pid_does_not_kill_other_process(self):
        # A process owned by this test only; its argv cannot match any state path.
        p = subprocess.Popen(['/bin/sleep', '30'])
        try:
            (self.state / 'duck-a.pid').write_text(str(p.pid))
            self.run_script('down', code=0, contains='not stopping pid')
            self.assertIsNone(p.poll())
            self.assertFalse((self.state / 'duck-a.pid').exists())
        finally:
            p.terminate()
            p.wait(timeout=3)

    def test_ctl_passthrough(self):
        self.run_script('ctl', 'health', code=0, contains='fifth')

    def test_drive_without_daemon(self):
        self.run_script('drive', code=1, contains='no duck')

    def test_unknown_command(self):
        self.run_script('does-not-exist', code=1, contains='unknown command')


if __name__ == '__main__':
    unittest.main(verbosity=2)
