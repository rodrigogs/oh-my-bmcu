#!/usr/bin/env python3
"""Check that verbatim copies of firmware code inside host tests still match src/.

Some host tests cannot link a firmware source file (it needs the CH32 SDK), so they carry a copy
of the code they exercise. Two kinds of copy are allowed:

    // ---- bambu_bus_ams.cpp at this commit: <what>, verbatim ----
    ...copied code...
    // ---- end of the bambu_bus_ams.cpp copy ----

must still appear, unchanged and contiguous, in src/<file> (line endings are ignored), and

    // ---- adapted from main.cpp: <what> ----

marks a copy that was changed for the host and cannot be checked; it is listed, not verified.
Any other '// ---- <file> at this commit:' marker is an error, as is a verbatim marker without its
end marker. Run from the repository root; exits non-zero on a drifted or malformed copy.
"""

import pathlib
import re
import sys

BLOCK = re.compile(
    r"// ---- (?P<file>\S+) at this commit: [^\n]*verbatim ----\n(?P<code>.*?)\n// ---- end of the (?P=file) copy ----",
    re.S,
)
AT_COMMIT = re.compile(r"^// ---- (\S+) at this commit:.*$", re.M)
ADAPTED = re.compile(r"^// ---- adapted from (\S+?):.*$", re.M)


def main():
    root = pathlib.Path.cwd()
    checked = 0
    errors = []
    adapted = []
    for test in sorted((root / "test").rglob("*")):
        if test.suffix not in (".c", ".cpp", ".h"):
            continue
        rel = test.relative_to(root)
        text = test.read_text().replace("\r\n", "\n")

        starts_ok = set()
        for m in BLOCK.finditer(text):
            checked += 1
            starts_ok.add(m.start())
            src_path = root / "src" / m.group("file")
            if not src_path.is_file():
                errors.append("%s: src/%s does not exist" % (rel, m.group("file")))
                continue
            src = src_path.read_bytes().decode().replace("\r\n", "\n")
            if m.group("code") not in src:
                errors.append("%s: the copy of src/%s no longer matches it" % (rel, m.group("file")))

        for m in AT_COMMIT.finditer(text):
            if m.start() not in starts_ok:
                line = text.count("\n", 0, m.start()) + 1
                errors.append("%s:%d: '%s' is not a checked verbatim block (mark it 'adapted from' or make "
                              "it verbatim with an end marker)" % (rel, line, m.group(0)[:80]))

        for m in ADAPTED.finditer(text):
            line = text.count("\n", 0, m.start()) + 1
            adapted.append("%s:%d (%s)" % (rel, line, m.group(1)))

    for e in errors:
        print("ERROR  " + e)
    for a in adapted:
        print("not checked (adapted copy)  " + a)
    print("%d verbatim cop%s checked, %d adapted cop%s listed, %d error%s"
          % (checked, "y" if checked == 1 else "ies", len(adapted), "y" if len(adapted) == 1 else "ies",
             len(errors), "" if len(errors) == 1 else "s"))
    sys.exit(1 if errors or checked == 0 else 0)


if __name__ == "__main__":
    main()
