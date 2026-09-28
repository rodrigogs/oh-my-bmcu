"""Fixture tests for scripts/check_fw_env.py.

It is a PlatformIO pre-script, not a standalone program: it calls the SCons-injected `Import`,
`Return` and `env` at module scope. Each test execs the real source with a fake `env` and a
controlled os.environ, so a break in its validation shows up here instead of only inside a
real `pio run`. Run with
    python3 -m unittest discover -s scripts/tests
"""

import contextlib
import io
import os
import unittest

from helpers import SCRIPTS

SOURCE = (SCRIPTS / "check_fw_env.py").read_text()

VALID_ENV = {
    "BMCU_DM_TWO_MICROSWITCH": "0",
    "BMCU_ONLINE_LED_FILAMENT_RGB": "1",
    "DBMCU_P1S": "0",
    "BMCU_SOFT_LOAD": "0",
    "BAMBU_BUS_AMS_NUM": "1",
    "AMS_RETRACT_LEN": "0.095f",
}


class _Returned(Exception):
    pass


class _Exited(Exception):
    pass


class _FakeEnv:
    def __init__(self, clean=False, integration_dump=False):
        self.clean = clean
        self.integration_dump = integration_dump
        self.exit_code = None

    def IsCleanTarget(self):
        return self.clean

    def IsIntegrationDump(self):
        return self.integration_dump

    def Exit(self, code):
        self.exit_code = code
        raise _Exited()


def run_prescript(env_vars, clean=False, integration_dump=False):
    """Runs the real check_fw_env.py source; returns (exit_code_or_None, stderr_text)."""
    fake_env = _FakeEnv(clean=clean, integration_dump=integration_dump)
    globals_ = {
        "Import": lambda *a, **k: None,
        "Return": lambda: (_ for _ in ()).throw(_Returned()),
        "env": fake_env,
        "__name__": "check_fw_env_under_test",
    }
    old_environ = dict(os.environ)
    os.environ.clear()
    os.environ.update(env_vars)
    stderr = io.StringIO()
    try:
        with contextlib.redirect_stderr(stderr):
            try:
                exec(compile(SOURCE, "check_fw_env.py", "exec"), globals_)
            except (_Returned, _Exited):
                pass
    finally:
        os.environ.clear()
        os.environ.update(old_environ)
    return fake_env.exit_code, stderr.getvalue()


class CheckFwEnv(unittest.TestCase):
    def test_valid_variant_env_passes(self):
        code, err = run_prescript(VALID_ENV)
        self.assertIsNone(code)
        self.assertEqual(err, "")

    def test_invalid_boolean_flag_fails(self):
        env = dict(VALID_ENV, BMCU_DM_TWO_MICROSWITCH="2")
        code, err = run_prescript(env)
        self.assertEqual(code, 1)
        self.assertIn("BMCU_DM_TWO_MICROSWITCH", err)

    def test_missing_ams_num_fails(self):
        env = {k: v for k, v in VALID_ENV.items() if k != "BAMBU_BUS_AMS_NUM"}
        code, err = run_prescript(env)
        self.assertEqual(code, 1)
        self.assertIn("BAMBU_BUS_AMS_NUM", err)

    def test_retract_len_out_of_range_fails(self):
        env = dict(VALID_ENV, AMS_RETRACT_LEN="3.0")
        code, err = run_prescript(env)
        self.assertEqual(code, 1)
        self.assertIn("AMS_RETRACT_LEN", err)

    def test_retract_len_boundary_values_pass(self):
        for value in ("0.05", "2.0", "0.095f"):
            with self.subTest(value=value):
                env = dict(VALID_ENV, AMS_RETRACT_LEN=value)
                code, err = run_prescript(env)
                self.assertIsNone(code, err)

    def test_p1s_and_soft_load_conflict_fails(self):
        env = dict(VALID_ENV, DBMCU_P1S="1", BMCU_SOFT_LOAD="1")
        code, err = run_prescript(env)
        self.assertEqual(code, 1)
        self.assertIn("DBMCU_P1S=1 and BMCU_SOFT_LOAD=1", err)

    def test_clean_target_skips_validation(self):
        code, err = run_prescript({}, clean=True)
        self.assertIsNone(code)
        self.assertEqual(err, "")

    def test_integration_dump_skips_validation(self):
        code, err = run_prescript({}, integration_dump=True)
        self.assertIsNone(code)
        self.assertEqual(err, "")


if __name__ == "__main__":
    unittest.main()
