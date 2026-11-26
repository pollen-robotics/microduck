#!/usr/bin/env python3
"""Check the real body launcher's arguments without building daemons or running MuJoCo."""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "duck-sim"
MARKER = '\ncase "${1:-up}" in'


class BodyArguments(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="duck-body-args-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bin = self.root / "bin"
        self.bin.mkdir()
        for name in ("dirname", "mkdir", "nohup", "mv", "sleep"):
            (self.bin / name).symlink_to(shutil.which(name))
        self.write_executable(self.bin / "python3", '[ -s "$DUCK_SIM_TEST_ARGS" ]\n')
        self.write_executable(self.bin / "uname", 'printf "%s\\n" "$DUCK_SIM_TEST_OS"\n')

    @staticmethod
    def write_executable(path, body):
        path.write_text("#!/bin/sh\nset -eu\n" + body)
        path.chmod(0o755)

    def launch(self, rl_name="rl", scene=None, cameras="", viewer=False, system="Linux"):
        rl = self.root / rl_name
        launcher_dir = rl / ".venv/bin"
        launcher_dir.mkdir(parents=True)
        args_file = self.root / "body-args"
        recorder = (
            'printf "%s\\000" "$0" "$@" > "$DUCK_SIM_TEST_ARGS.new"\n'
            'mv "$DUCK_SIM_TEST_ARGS.new" "$DUCK_SIM_TEST_ARGS"\n'
        )
        for name in ("python", "mjpython"):
            self.write_executable(launcher_dir / name, recorder)

        # Load only the function definitions, then call start_body. The real CLI
        # dispatcher would also build daemons, fetch models and start the full stack.
        definitions, marker, _ = SCRIPT.read_text().rpartition(MARKER)
        self.assertTrue(marker, "duck-sim's command dispatcher must be identifiable")
        env = {
            **os.environ,
            "PATH": str(self.bin),
            "DUCK_SIM_STATE": str(self.root / "state"),
            "DUCK_SIM_RL": str(rl),
            "DUCK_SIM_PORT": "7801",
            "DUCK_SIM_DUCKS": "2",
            "DUCK_SIM_FRAME_PORT": "7901",
            "DUCK_SIM_CAMERAS": cameras,
            "DUCK_SIM_VIEWER": "1" if viewer else "0",
            "DUCK_SIM_KEYFRAME": "SIT",
            "DUCK_SIM_SCENE": scene if scene is not None else "backlash",
            "DUCK_SIM_TEST_ARGS": str(args_file),
            "DUCK_SIM_TEST_OS": system,
        }
        result = subprocess.run(
            ["/bin/sh", "-c", definitions + "\nstart_body\n", str(SCRIPT)],
            cwd=self.root,
            env=env,
            capture_output=True,
            text=True,
            timeout=5,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        args = [os.fsdecode(arg) for arg in args_file.read_bytes().split(b"\0")[:-1]]
        launcher = "mjpython" if viewer and system == "Darwin" else "python"
        expected = [
            str(launcher_dir / launcher), "-m", "mjlab_microduck.sim.body_server",
            "--port", "7801", "--ducks", "2", "--keyframe", "SIT",
        ]
        if not viewer:
            expected += ["--headless"]
        expected += ["--scene", scene if scene is not None else str(
            rl / "src/mjlab_microduck/robot/microduck/scene_backlash.xml"
        )]
        if cameras:
            expected += ["--cameras", cameras, "--frame-port", "7901", "--camera-fps", "30"]
        self.assertEqual(args, expected)

    def test_default_scene(self):
        self.launch()

    def test_rl_checkout_path_with_spaces(self):
        self.launch(rl_name="rl checkout")

    def test_custom_scene_path_with_spaces(self):
        scene = self.root / "custom course.xml"
        scene.touch()
        self.launch(scene=str(scene))

    def test_custom_scene_path_is_not_a_glob(self):
        # The literal path must not expand to the neighbouring file's name.
        scene = self.root / "course[1].xml"
        scene.touch()
        (self.root / "course1.xml").touch()
        self.launch(scene=str(scene))

    def test_camera_options_are_preserved(self):
        self.launch(cameras="a,c")

    def test_macos_viewer_uses_mjpython(self):
        self.launch(viewer=True, system="Darwin")


if __name__ == "__main__":
    unittest.main(verbosity=2)
