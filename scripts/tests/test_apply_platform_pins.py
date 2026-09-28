"""Fixture tests for scripts/apply_platform_pins.py. Run with
    python3 -m unittest discover -s scripts/tests
"""

import configparser
import pathlib
import tempfile
import unittest

from helpers import run_script, write

SCRIPT = "apply_platform_pins.py"


def read_ini(path):
    cfg = configparser.RawConfigParser(strict=False, inline_comment_prefixes=(";",))
    cfg.optionxform = str
    cfg.read(path)
    return cfg


class ApplyPlatformPins(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = pathlib.Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def test_pinned_keys_are_copied_from_source(self):
        source = self.tmp / "source.ini"
        target = self.tmp / "target.ini"
        write(source, "[env:base]\nplatform = riscv-ch32v@1.0.0\nplatform_packages = toolchain-riscv-mac@1.2.3\n")
        write(target, "[env:base]\nplatform = old-platform@0.0.1\nboard = genericCH32V203C8T6\n")
        rc, out, _err = run_script(SCRIPT, [str(source), str(target)])
        self.assertEqual(rc, 0, out)
        self.assertIn("platform = riscv-ch32v@1.0.0", out)
        self.assertIn("platform_packages = toolchain-riscv-mac@1.2.3", out)
        cfg = read_ini(target)
        self.assertEqual(cfg.get("env:base", "platform"), "riscv-ch32v@1.0.0")
        self.assertEqual(cfg.get("env:base", "platform_packages"), "toolchain-riscv-mac@1.2.3")
        # A key the pinning does not touch stays as the target had it.
        self.assertEqual(cfg.get("env:base", "board"), "genericCH32V203C8T6")

    def test_key_missing_from_source_is_removed_from_target(self):
        source = self.tmp / "source.ini"
        target = self.tmp / "target.ini"
        write(source, "[env:base]\nplatform = riscv-ch32v@1.0.0\n")
        write(target, "[env:base]\nplatform = old-platform@0.0.1\nplatform_packages = toolchain-riscv-mac@1.2.3\n")
        rc, _out, _err = run_script(SCRIPT, [str(source), str(target)])
        self.assertEqual(rc, 0)
        cfg = read_ini(target)
        self.assertEqual(cfg.get("env:base", "platform"), "riscv-ch32v@1.0.0")
        self.assertFalse(cfg.has_option("env:base", "platform_packages"))

    def test_wrong_argument_count_prints_usage_and_fails(self):
        rc, out, err = run_script(SCRIPT, [str(self.tmp / "only_one.ini")])
        self.assertNotEqual(rc, 0)
        self.assertIn("apply_platform_pins.py", out + err)


if __name__ == "__main__":
    unittest.main()
