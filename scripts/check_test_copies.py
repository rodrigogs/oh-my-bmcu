#!/usr/bin/env python3
"""Check that verbatim copies of firmware code inside host tests still match src/.

Some host tests cannot link a firmware source file (it needs the CH32 SDK), so they carry a copy
of the code they exercise. Two kinds of copy are allowed:

    // ---- bambu_bus_ams.cpp at this commit: <what>, verbatim ----
    ...copied code...
    // ---- end of the bambu_bus_ams.cpp copy ----

must still appear, unchanged, contiguous and as whole lines, in src/<file> (line endings are
ignored), and

    // ---- adapted from main.cpp: <what> ----

marks a copy that was changed for the host and cannot be checked; it is listed, not verified (the
file it names must exist, in src/ or from the repository root).
Any other '// ---- <file> at this commit:' marker is an error, as is a verbatim marker without its
end marker. So is a test that defines, outside a verbatim block and at file scope, a constant or
macro a src/ file also defines at file scope (an UPPER_CASE or kName #define, object-like or
function-like, or a const or constexpr variable): it should include the header or copy the
definition verbatim. A src/ build-flag default ('#ifndef X' then '#define X') does not count, so a
test may still pick a configuration; a test's own '#ifndef X' does not exempt a name src/ defines
unconditionally. Not found this way (mark such copies): a copy under another name, a constant in a
namespace or an enum, a declaration whose name is not on the line of its 'const', a const
initialised with braces or parentheses instead of '=', and a second declarator
('const int a = 1, B = 2;'). Run from the repository root; exits non-zero on a drifted
or malformed copy.
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

COMMENT_OR_STRING = re.compile(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])*\'', re.S)
CONST_NAME = re.compile(r"^(?:[A-Z][A-Z0-9_]*|k[A-Z]\w*)$")
DEFINE = re.compile(r"^\s*#\s*define\s+([A-Za-z_]\w*)")
IFNDEF = re.compile(r"^\s*#\s*ifndef\s+([A-Za-z_]\w*)")
CONST_DECL = re.compile(r"\b(?:constexpr|const)\b[^;{}()=]*?\b([A-Za-z_]\w*)\s*(?:\[[^\]]*\]\s*)*=")


def blank(text):
    """Comments and string literals replaced by spaces, newlines kept."""
    return COMMENT_OR_STRING.sub(lambda m: re.sub(r"[^\n]", " ", m.group(0)), text)


def file_scope_constants(text, defaults_count=False):
    """{name: line} of the constants and macros text defines at file scope (brace depth 0). A
    build-flag default ('#ifndef X' then '#define X') is left out unless defaults_count."""
    found = {}
    depth = 0
    ifndef = None
    continued = False
    for number, line in enumerate(blank(text).split("\n"), 1):
        # The rest of a multi-line directive (a macro body) is neither a declaration nor a brace.
        if continued:
            continued = line.rstrip().endswith("\\")
            continue
        continued = line.lstrip().startswith("#") and line.rstrip().endswith("\\")
        m = IFNDEF.match(line)
        if m:
            ifndef = m.group(1)
            continue
        m = DEFINE.match(line)
        if m:
            if defaults_count or m.group(1) != ifndef:
                found.setdefault(m.group(1), number)
            ifndef = None
            continue
        if line.strip():
            ifndef = None
        if not line.lstrip().startswith("#"):
            for d in CONST_DECL.finditer(line):
                if depth + line.count("{", 0, d.start()) - line.count("}", 0, d.start()) == 0:
                    found.setdefault(d.group(1), number)
        depth += line.count("{") - line.count("}")
    return {n: l for n, l in found.items() if CONST_NAME.match(n)}


def main():
    root = pathlib.Path.cwd()
    checked = 0
    errors = []
    adapted = []

    src_constants = {}
    for src_file in sorted((root / "src").rglob("*")):
        if src_file.suffix in (".c", ".cpp", ".h"):
            for name in file_scope_constants(src_file.read_bytes().decode().replace("\r\n", "\n")):
                src_constants.setdefault(name, str(src_file.relative_to(root)))
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
            # Whole lines: a copy must not match text that src/ only has inside a longer line or a comment.
            if "\n" + m.group("code") + "\n" not in "\n" + src + "\n":
                errors.append("%s: the copy of src/%s no longer matches it" % (rel, m.group("file")))

        for m in AT_COMMIT.finditer(text):
            if m.start() not in starts_ok:
                line = text.count("\n", 0, m.start()) + 1
                errors.append("%s:%d: '%s' is not a checked verbatim block (mark it 'adapted from' or make "
                              "it verbatim with an end marker)" % (rel, line, m.group(0)[:80]))

        for m in ADAPTED.finditer(text):
            line = text.count("\n", 0, m.start()) + 1
            adapted.append("%s:%d (%s)" % (rel, line, m.group(1)))
            if not ((root / "src" / m.group(1)).is_file() or (root / m.group(1)).is_file()):
                errors.append("%s:%d: 'adapted from %s' names no file in src/ or the repository"
                              % (rel, line, m.group(1)))

        # Verbatim blocks are checked above; blank them out, keeping the line numbers.
        outside = BLOCK.sub(lambda b: re.sub(r"[^\n]", " ", b.group(0)), text)
        # A test's own '#ifndef X' default counts: only src/ decides what is a build flag.
        test_constants = file_scope_constants(outside, defaults_count=True)
        for name, line in sorted(test_constants.items(), key=lambda x: x[1]):
            if name in src_constants:
                errors.append("%s:%d: defines %s, which %s also defines: include its header or copy it "
                              "in a verbatim block" % (rel, line, name, src_constants[name]))

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
