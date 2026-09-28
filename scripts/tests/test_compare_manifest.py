"""Fixture tests for scripts/compare_manifest.py. Run with
    python3 -m unittest discover -s scripts/tests
"""

import pathlib
import tempfile
import unittest

from helpers import run_script, write

SCRIPT = "compare_manifest.py"
LINE = "%s 0 12 %s\n"


class CompareManifest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = pathlib.Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def manifest(self, name, entries):
        path = self.tmp / name
        write(path, "".join(LINE % (sha, rel) for sha, rel in entries))
        return path

    def test_identical_manifests_pass(self):
        built = self.manifest("built.txt", [("a" * 64, "SOLO/x.bin")])
        ref = self.manifest("ref.txt", [("a" * 64, "SOLO/x.bin")])
        rc, out, _err = run_script(SCRIPT, [str(built), str(ref)])
        self.assertEqual(rc, 0, out)
        self.assertIn("OK        SOLO/x.bin", out)
        self.assertIn("1/1 images identical", out)

    def test_hash_mismatch_fails(self):
        built = self.manifest("built.txt", [("a" * 64, "SOLO/x.bin")])
        ref = self.manifest("ref.txt", [("b" * 64, "SOLO/x.bin")])
        rc, out, _err = run_script(SCRIPT, [str(built), str(ref)])
        self.assertEqual(rc, 1)
        self.assertIn("MISMATCH  SOLO/x.bin", out)

    def test_missing_from_reference_fails(self):
        built = self.manifest("built.txt", [("a" * 64, "SOLO/x.bin")])
        ref = self.manifest("ref.txt", [])
        rc, out, _err = run_script(SCRIPT, [str(built), str(ref)])
        self.assertEqual(rc, 1)
        self.assertIn("reference=<missing>", out)

    def test_no_bin_entries_fails(self):
        built = self.manifest("built.txt", [])
        ref = self.manifest("ref.txt", [])
        rc, _out, err = run_script(SCRIPT, [str(built), str(ref)])
        self.assertEqual(rc, 1)
        self.assertIn("no .bin entries", err)

    def test_expect_count_mismatch_fails_even_if_hashes_match(self):
        built = self.manifest("built.txt", [("a" * 64, "SOLO/x.bin")])
        ref = self.manifest("ref.txt", [("a" * 64, "SOLO/x.bin")])
        rc, out, _err = run_script(SCRIPT, [str(built), str(ref), "--expect-count", "2"])
        self.assertEqual(rc, 1)
        self.assertIn("expected 2 built images, got 1", out)

    def test_expect_count_match_passes(self):
        built = self.manifest("built.txt", [("a" * 64, "SOLO/x.bin")])
        ref = self.manifest("ref.txt", [("a" * 64, "SOLO/x.bin")])
        rc, out, _err = run_script(SCRIPT, [str(built), str(ref), "--expect-count", "1"])
        self.assertEqual(rc, 0, out)


if __name__ == "__main__":
    unittest.main()
