"""Fixture tests for scripts/check_test_copies.py: each builds a tiny src/+test/ tree in a temp
dir (never a committed tree) and runs the real script against it, so a break in the guard shows up
as a failing case here instead of only on the real 28 copies. Run with
    python3 -m unittest discover -s scripts/tests
"""

import pathlib
import tempfile
import unittest

from helpers import run_script, write

SCRIPT = "check_test_copies.py"


def base(tmp):
    """One good verbatim copy, so `checked` is never 0 (the script also fails empty-handed)."""
    write(tmp / "src/base.cpp", "int base_fn(void) {\n    return 1;\n}\n")
    write(
        tmp / "test/test_base/test_base.c",
        "// ---- base.cpp at this commit: base_fn, verbatim ----\n"
        "int base_fn(void) {\n    return 1;\n}\n"
        "// ---- end of the base.cpp copy ----\n",
    )


class CheckTestCopies(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = pathlib.Path(self._tmp.name)
        (self.tmp / "test").mkdir()
        (self.tmp / "src").mkdir()

    def tearDown(self):
        self._tmp.cleanup()

    def run_guard(self, args=()):
        return run_script(SCRIPT, args, cwd=self.tmp)

    # -- baseline: the fixture builder itself must pass, or every other test's "should pass"
    # assertions are meaningless.

    def test_baseline_passes(self):
        base(self.tmp)
        rc, out, _err = self.run_guard()
        self.assertEqual(rc, 0, out)
        self.assertNotIn("ERROR", out)

    # -- verbatim copies --

    def test_drifted_verbatim_copy_is_an_error(self):
        base(self.tmp)
        write(
            self.tmp / "src/drift.cpp",
            "int drift_fn(void) {\n    return 1;\n}\n",
        )
        write(
            self.tmp / "test/test_drift/test_drift.c",
            "// ---- drift.cpp at this commit: drift_fn, verbatim ----\n"
            "int drift_fn(void) {\n    return 2;\n}\n"
            "// ---- end of the drift.cpp copy ----\n",
        )
        rc, out, _err = self.run_guard()
        self.assertEqual(rc, 1)
        self.assertIn("no longer matches it", out)

    def test_missing_end_marker_is_an_error(self):
        base(self.tmp)
        write(self.tmp / "src/nomark.cpp", "int nomark_fn(void) {\n    return 1;\n}\n")
        write(
            self.tmp / "test/test_nomark/test_nomark.c",
            "// ---- nomark.cpp at this commit: nomark_fn, verbatim ----\n"
            "int nomark_fn(void) {\n    return 1;\n}\n",
        )
        rc, out, _err = self.run_guard()
        self.assertEqual(rc, 1)
        self.assertIn("is not a checked verbatim block", out)

    def test_marker_without_verbatim_keyword_is_an_error(self):
        base(self.tmp)
        write(self.tmp / "src/typo.cpp", "int typo_fn(void) {\n    return 1;\n}\n")
        write(
            self.tmp / "test/test_typo/test_typo.c",
            "// ---- typo.cpp at this commit: typo_fn, copied ----\n"
            "int typo_fn(void) {\n    return 1;\n}\n"
            "// ---- end of the typo.cpp copy ----\n",
        )
        rc, out, _err = self.run_guard()
        self.assertEqual(rc, 1)
        self.assertIn("is not a checked verbatim block", out)

    def test_crlf_copy_still_matches(self):
        base(self.tmp)
        write(self.tmp / "src/crlf.cpp", "int crlf_fn(void) {\n    return 1;\n}\n")
        write(
            self.tmp / "test/test_crlf/test_crlf.c",
            (
                "// ---- crlf.cpp at this commit: crlf_fn, verbatim ----\r\n"
                "int crlf_fn(void) {\r\n    return 1;\r\n}\r\n"
                "// ---- end of the crlf.cpp copy ----\r\n"
            ),
        )
        rc, out, _err = self.run_guard()
        self.assertEqual(rc, 0, out)
        self.assertNotIn("ERROR", out)

    # -- redefined constants --

    def test_redefined_src_constant_is_an_error(self):
        base(self.tmp)
        write(self.tmp / "src/limits.h", "#define FOO_LIMIT 5\n")
        write(self.tmp / "test/test_limits/test_limits.c", "#define FOO_LIMIT 7\n")
        rc, out, _err = self.run_guard()
        self.assertEqual(rc, 1)
        self.assertIn("FOO_LIMIT", out)
        self.assertIn("also defines", out)

    def test_src_build_flag_default_may_be_picked_by_a_test(self):
        base(self.tmp)
        write(
            self.tmp / "src/limits.h",
            "#ifndef DEFAULT_X\n#define DEFAULT_X 5\n#endif\n",
        )
        write(self.tmp / "test/test_limits/test_limits.c", "#define DEFAULT_X 9\n")
        rc, out, _err = self.run_guard()
        self.assertEqual(rc, 0, out)
        self.assertNotIn("ERROR", out)

    def test_tests_own_ifndef_does_not_exempt_an_unconditional_src_constant(self):
        base(self.tmp)
        write(self.tmp / "src/limits.h", "#define BAZ_LIMIT 3\n")
        write(
            self.tmp / "test/test_limits/test_limits.c",
            "#ifndef BAZ_LIMIT\n#define BAZ_LIMIT 9\n#endif\n",
        )
        rc, out, _err = self.run_guard()
        self.assertEqual(rc, 1)
        self.assertIn("BAZ_LIMIT", out)

    # -- unregistered test_* (this item's new check) --

    def test_defined_but_never_run_test_is_an_error(self):
        base(self.tmp)
        write(
            self.tmp / "test/test_orphan/test_orphan.c",
            "static void test_orphan_case(void)\n{\n}\n"
            "static void test_wired_case(void)\n{\n}\n"
            "int main(void) {\n    RUN_TEST(test_wired_case);\n    return 0;\n}\n",
        )
        rc, out, _err = self.run_guard()
        self.assertEqual(rc, 1)
        self.assertIn("test_orphan_case", out)
        self.assertIn("never passed to RUN_TEST", out)

    def test_every_test_run_is_not_an_error(self):
        base(self.tmp)
        write(
            self.tmp / "test/test_wired/test_wired.c",
            "static void test_wired_case(void)\n{\n}\n"
            "int main(void) {\n    RUN_TEST(test_wired_case);\n    return 0;\n}\n",
        )
        rc, out, _err = self.run_guard()
        self.assertEqual(rc, 0, out)
        self.assertNotIn("ERROR", out)

    # -- anchor/lock (added by the previous item; the guard tool suite must cover it too) --

    def test_relock_then_clean_run_passes(self):
        base(self.tmp)
        write(self.tmp / "src/locked.cpp", "int locked_fn(void) {\n    return 1;\n}\n")
        write(
            self.tmp / "test/test_locked/test_locked.c",
            "// ---- adapted from locked.cpp: a model ----\n"
            "// ---- anchor: locked_fn ----\n"
            "int fake_locked_fn(void) { return 1; }\n",
        )
        rc, out, _err = self.run_guard(["--relock"])
        self.assertEqual(rc, 0, out)
        self.assertIn("wrote", out)
        rc, out, _err = self.run_guard()
        self.assertEqual(rc, 0, out)
        self.assertNotIn("ERROR", out)

    def test_relock_is_idempotent(self):
        base(self.tmp)
        write(self.tmp / "src/locked.cpp", "int locked_fn(void) {\n    return 1;\n}\n")
        write(
            self.tmp / "test/test_locked/test_locked.c",
            "// ---- adapted from locked.cpp: a model ----\n"
            "// ---- anchor: locked_fn ----\n"
            "int fake_locked_fn(void) { return 1; }\n",
        )
        self.run_guard(["--relock"])
        first = (self.tmp / "test/adapted.lock").read_text()
        rc, out, _err = self.run_guard(["--relock"])
        self.assertEqual(rc, 0, out)
        second = (self.tmp / "test/adapted.lock").read_text()
        self.assertEqual(first, second)

    def test_drifted_locked_region_is_an_error(self):
        base(self.tmp)
        write(self.tmp / "src/locked.cpp", "int locked_fn(void) {\n    return 1;\n}\n")
        write(
            self.tmp / "test/test_locked/test_locked.c",
            "// ---- adapted from locked.cpp: a model ----\n"
            "// ---- anchor: locked_fn ----\n"
            "int fake_locked_fn(void) { return 1; }\n",
        )
        rc, _out, _err = self.run_guard(["--relock"])
        self.assertEqual(rc, 0)
        write(self.tmp / "src/locked.cpp", "int locked_fn(void) {\n    return 2;\n}\n")
        rc, out, _err = self.run_guard()
        self.assertEqual(rc, 1)
        self.assertIn("changed since it was locked", out)

    def test_anchor_that_no_longer_resolves_is_an_error(self):
        base(self.tmp)
        write(self.tmp / "src/locked.cpp", "int locked_fn(void) {\n    return 1;\n}\n")
        write(
            self.tmp / "test/test_locked/test_locked.c",
            "// ---- adapted from locked.cpp: a model ----\n"
            "// ---- anchor: locked_fn ----\n"
            "int fake_locked_fn(void) { return 1; }\n",
        )
        self.run_guard(["--relock"])
        # The symbol the anchor names is gone: the anchor no longer resolves.
        write(self.tmp / "src/locked.cpp", "int renamed_fn(void) {\n    return 1;\n}\n")
        rc, out, _err = self.run_guard()
        self.assertEqual(rc, 1)
        self.assertIn("does not resolve", out)

    def test_prose_marker_without_anchor_is_reported_unlocked(self):
        base(self.tmp)
        write(self.tmp / "src/prose.cpp", "int prose_fn(void) {\n    return 1;\n}\n")
        write(
            self.tmp / "test/test_prose/test_prose.c",
            "// ---- adapted from prose.cpp: a model, no anchor needed ----\n"
            "int fake_prose_fn(void) { return 1; }\n",
        )
        rc, out, _err = self.run_guard()
        self.assertEqual(rc, 0, out)
        self.assertIn("not checked (adapted copy)", out)


if __name__ == "__main__":
    unittest.main()
