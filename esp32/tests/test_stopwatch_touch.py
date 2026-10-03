# Copyright (c) Dehong Hao. SPDX-License-Identifier: Apache-2.0
"""Run the vendored M5Stack touch driver against CST820B packets and I2C failures."""
from __future__ import annotations

import os
import shlex
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class StopWatchTouchTest(unittest.TestCase):
    def test_touch_reports_and_failed_probe(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / "stopwatch-touch"
            command = [
                *shlex.split(os.environ.get("CXX", "c++")),
                "-std=c++17", "-Wall", "-Wextra", "-Werror",
                "-I", str(ROOT / "tests/stopwatch_fakes"),
                "-I", str(ROOT / "components/muse/boards"),
                str(ROOT / "tests/stopwatch_touch_harness.cpp"),
                str(ROOT / "components/muse/boards/cst820.cpp"),
                "-o", str(binary),
            ]
            result = subprocess.run(command, capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
