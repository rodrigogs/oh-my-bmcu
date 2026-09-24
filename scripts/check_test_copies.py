#!/usr/bin/env python3
"""Check that verbatim copies of firmware code inside host tests still match src/.

Some host tests cannot link a firmware source file (it needs the CH32 SDK), so they carry a copy
of the functions they exercise, marked like this:

    // ---- bambu_bus_ams.cpp at this commit: <what>, verbatim ----
    ...copied code...
    // ---- end of the bambu_bus_ams.cpp copy ----

Every such block must still appear, unchanged and contiguous, in src/<file> (line endings are
ignored). Run from the repository root; exits non-zero if a copy has drifted.
"""

import pathlib
import re
import sys

BLOCK = re.compile(
    r"// ---- (?P<file>\S+) at this commit: [^\n]*verbatim ----\n(?P<code>.*?)\n// ---- end of the (?P=file) copy ----",
    re.S,
)


def main():
    root = pathlib.Path.cwd()
    checked = 0
    drifted = []
    for test in sorted((root / "test").rglob("*")):
        if test.suffix not in (".c", ".cpp", ".h"):
            continue
        text = test.read_text().replace("\r\n", "\n")
        for m in BLOCK.finditer(text):
            checked += 1
            src_path = root / "src" / m.group("file")
            if not src_path.is_file():
                drifted.append("%s: src/%s does not exist" % (test.relative_to(root), m.group("file")))
                continue
            src = src_path.read_bytes().decode().replace("\r\n", "\n")
            if m.group("code") not in src:
                drifted.append("%s: the copy of src/%s no longer matches it" % (test.relative_to(root), m.group("file")))

    for d in drifted:
        print("DRIFT  " + d)
    print("%d verbatim cop%s checked, %d drifted" % (checked, "y" if checked == 1 else "ies", len(drifted)))
    sys.exit(1 if drifted or checked == 0 else 0)


if __name__ == "__main__":
    main()
