#!/usr/bin/env python3
"""Check that copies of firmware code inside host tests still match src/.

Some host tests cannot link a firmware source file (it needs the CH32 SDK), so they carry a copy
of the code they exercise. Two kinds of copy are allowed:

    // ---- bambu_bus_ams.cpp at this commit: <what>, verbatim ----
    ...copied code...
    // ---- end of the bambu_bus_ams.cpp copy ----

must still appear, unchanged, contiguous and as whole lines, in src/<file> (line endings are
ignored), and

    // ---- adapted from main.cpp: <what> ----
    // ---- anchor: main from /while \(1\)/ to /ams_nvm_save_run\(\);/ ----

marks a copy that was changed for the host, so it cannot be compared (the file it names must exist,
in src/ or from the repository root). The anchor lines right under it, if any, name the regions of
src/<file> it models: a function, class, struct, enum, #define or file-scope variable defined once
in that file (from the line that names it to its closing brace, its ';' or the end of the #define),
optionally narrowed to the lines from the first one matching 'from /<regex>/' to the first one at or
after it matching 'to /<regex>/' (Python regexes searched in each line, comments included). No line
numbers: every edit above the region would move them. The hash of each region (line endings and
trailing whitespace ignored) must match its line in test/adapted.lock, so an edit inside a region
fails here, naming the harnesses that model it, until they have been reviewed and the lock
rewritten with

    python3 scripts/check_test_copies.py --relock

which writes the hash of every anchor as the tree has them (run again, it changes nothing). An
anchor that no longer resolves (its symbol gone or defined more than once, a regex that matches no
line) is an error, and --relock then writes nothing; so are an anchor line not under an adapted
marker, an anchor missing from the lock and a lock line no anchor uses. An adapted marker without
an anchor is accepted and counted as unlocked. A lock only says that the models were looked at
after their region last changed, not that they are right; it does not see an edit outside the
region that changes what the region does (a constant or helper defined elsewhere, a return type on
the line above the name, the branch of an #if the region is not in).
Any other '// ---- <file> at this commit:' marker is an error, as is a verbatim marker without its
end marker. So is a test that defines, outside a verbatim block and at file scope, a constant or
macro a src/ file also defines at file scope (an UPPER_CASE or kName #define, object-like or
function-like, or a const or constexpr variable): it should include the header or copy the
definition verbatim. A src/ build-flag default ('#ifndef X' then '#define X') does not count, so a
test may still pick a configuration; a test's own '#ifndef X' does not exempt a name src/ defines
unconditionally. Not found this way (mark such copies): a copy under another name, a constant in a
namespace or an enum, a declaration whose name is not on the line of its 'const', a const
initialised with braces or parentheses instead of '=', and a second declarator
('const int a = 1, B = 2;'). Also checked: every `void test_*(void)` (optionally 'static')
defined in a test/ file is passed to a RUN_TEST() in that same file, so a case written but
never wired in is reported instead of silently never running. Run from the repository root;
exits non-zero on a drifted or malformed copy or a changed anchored region.
"""

import argparse
import hashlib
import pathlib
import re
import sys

BLOCK = re.compile(
    r"// ---- (?P<file>\S+) at this commit: [^\n]*verbatim ----\n(?P<code>.*?)\n// ---- end of the (?P=file) copy ----",
    re.S,
)
AT_COMMIT = re.compile(r"^// ---- (\S+) at this commit:.*$", re.M)
ADAPTED = re.compile(r"^// ---- adapted from (\S+?):.*$", re.M)
ANCHOR = re.compile(r"^// ---- anchor: (.*) ----$", re.M)
ANCHOR_SPEC = re.compile(r"^(?P<symbol>[A-Za-z_]\w*)(?: from /(?P<start>.+?)/)?(?: to /(?P<end>.+?)/)?$")
TEST_DEF = re.compile(r"^(?:static\s+)?void\s+(test_\w+)\s*\(\s*void\s*\)\s*$", re.M)
RUN_TEST_CALL = re.compile(r"\bRUN_TEST\s*\(\s*(test_\w+)\s*\)")

# What follows a symbol's name where it is defined: a function's body after its parameters, a type's
# keyword before it, a file-scope variable's size, initialiser or ';' after it.
FUNCTION_BODY = re.compile(r"\s*(?:(?:const|noexcept|override)\b\s*)*(?::[^;{}]*)?\{")
TYPE_HEAD = re.compile(r"\b(?:class|struct|union|enum(?:\s+class)?)\s+(?:alignas\s*\([^)]*\)\s*)?$")
VARIABLE_TAIL = re.compile(r"\s*(?:\[[^\]]*\]\s*)*(?:=(?!=)|\{|;)")

LOCK = "test/adapted.lock"
RELOCK = "python3 scripts/check_test_copies.py --relock"
LOCK_HEADER = """\
# The src/ regions that the 'adapted from' models in test/ are locked to: the sha256 of each region
# (line endings and trailing whitespace ignored), its file and its anchor. Written by
#     %s
# once the models of every region that changed have been reviewed (scripts/check_test_copies.py).
""" % RELOCK

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


def closing(code, i):
    """Offset just past the bracket that closes the one at code[i]."""
    pair = {"(": ")", "{": "}"}[code[i]]
    depth = 0
    for j in range(i, len(code)):
        if code[j] == code[i]:
            depth += 1
        elif code[j] == pair:
            depth -= 1
            if depth == 0:
                return j + 1
    return len(code)


def statement_end(code, i):
    """Offset just past the ';' that ends the declaration at i, outside braces and parentheses."""
    depth = 0
    for j in range(i, len(code)):
        if code[j] in "({":
            depth += 1
        elif code[j] in ")}":
            depth -= 1
        elif code[j] == ";" and depth == 0:
            return j + 1
    return len(code)


def definitions(code, symbol):
    """[(first, last)] 0-based lines of what defines symbol in code (comments and strings blanked):
    a #define, a function (a class member too), a class, struct, union or enum, or a variable at
    file scope. Prototypes, forward and extern declarations and uses are left out."""
    lines = code.split("\n")
    found = []
    for m in re.finditer(r"\b%s\b" % re.escape(symbol), code):
        first = code.count("\n", 0, m.start())
        head = code[code.rfind("\n", 0, m.start()) + 1:m.start()]
        params = re.compile(r"\s*\(").match(code, m.end())
        end = None
        if head.lstrip().startswith("#"):
            if re.fullmatch(r"\s*#\s*define\s+", head):
                last = first
                while lines[last].rstrip().endswith("\\") and last + 1 < len(lines):
                    last += 1
                found.append((first, last))
            continue
        if params:
            body = FUNCTION_BODY.match(code, closing(code, params.end() - 1))
            if body:
                end = closing(code, body.end() - 1)
        elif TYPE_HEAD.search(head):
            brace = code.find("{", m.end())
            if brace != -1 and ";" not in code[m.end():brace]:
                end = statement_end(code, m.start())
        elif ("extern" not in head.split()
              and code.count("{", 0, m.start()) == code.count("}", 0, m.start())
              and code.count("(", 0, m.start()) == code.count(")", 0, m.start())
              and VARIABLE_TAIL.match(code, m.end())):
            end = statement_end(code, m.start())
        if end is not None:
            found.append((first, code.count("\n", 0, end - 1)))
    # A class's constructors are inside it.
    return [s for s in found if not any(o != s and o[0] <= s[0] and s[1] <= o[1] for o in found)]


def anchor_region(src, spec):
    """(first, last) 0-based lines of the region an anchor spec names in the src text, or the
    reason it does not resolve."""
    m = ANCHOR_SPEC.match(spec)
    if not m:
        return "'%s' is not '<symbol>[ from /<regex>/][ to /<regex>/]'" % spec
    spans = definitions(blank(src), m.group("symbol"))
    if len(spans) != 1:
        return "%s is defined %s there" % (m.group("symbol"), "%d times" % len(spans) if spans else "nowhere")
    first, last = spans[0]
    lines = src.split("\n")
    for part in ("start", "end"):
        if m.group(part) is None:
            continue
        try:
            rx = re.compile(m.group(part))
        except re.error as e:
            return "/%s/ is not a regex (%s)" % (m.group(part), e)
        hit = next((i for i in range(first, last + 1) if rx.search(lines[i])), None)
        if hit is None:
            return "none of lines %d-%d matches /%s/" % (first + 1, last + 1, m.group(part))
        first, last = (hit, last) if part == "start" else (first, hit)
    return first, last


def region_hash(src, first, last):
    """sha256 of lines first..last of src, trailing whitespace ignored."""
    return hashlib.sha256("\n".join(l.rstrip() for l in src.split("\n")[first:last + 1]).encode()).hexdigest()


def read_lock(path):
    """{(file, spec): sha256} from the lock file; empty if there is none."""
    lock = {}
    if path.is_file():
        for line in path.read_text().replace("\r\n", "\n").split("\n"):
            if line.strip() and not line.startswith("#"):
                digest, file, spec = line.split(" ", 2)
                lock[(file, spec)] = digest
    return lock


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--relock", action="store_true",
                        help="rewrite %s from the anchored src/ regions as they are now (once their "
                             "models have been reviewed)" % LOCK)
    relock = parser.parse_args().relock
    root = pathlib.Path.cwd()
    checked = 0
    errors = []
    adapted = []   # (where, file, [anchor spec])
    anchors = {}   # (src/file, spec): [the adapted markers that name it]

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

        anchored = set()
        for m in ADAPTED.finditer(text):
            line = text.count("\n", 0, m.start()) + 1
            where = "%s:%d" % (rel, line)
            specs = []
            # Its anchors: the anchor lines right under it.
            a = ANCHOR.match(text, m.end() + 1)
            while a:
                anchored.add(a.start())
                specs.append(a.group(1))
                a = ANCHOR.match(text, a.end() + 1)
            if not ((root / "src" / m.group(1)).is_file() or (root / m.group(1)).is_file()):
                errors.append("%s: 'adapted from %s' names no file in src/ or the repository" % (where, m.group(1)))
                specs = []
            elif specs and not (root / "src" / m.group(1)).is_file():
                errors.append("%s: anchors resolve in src/ only, not in %s" % (where, m.group(1)))
                specs = []
            for spec in specs:
                anchors.setdefault(("src/" + m.group(1), spec), []).append(where)
            adapted.append((where, m.group(1), specs))

        for a in ANCHOR.finditer(text):
            if a.start() not in anchored:
                errors.append("%s:%d: '%s' is not under an 'adapted from' marker"
                              % (rel, text.count("\n", 0, a.start()) + 1, a.group(0)[:80]))

        # Verbatim blocks are checked above; blank them out, keeping the line numbers.
        outside = BLOCK.sub(lambda b: re.sub(r"[^\n]", " ", b.group(0)), text)
        # A test's own '#ifndef X' default counts: only src/ decides what is a build flag.
        test_constants = file_scope_constants(outside, defaults_count=True)
        for name, line in sorted(test_constants.items(), key=lambda x: x[1]):
            if name in src_constants:
                errors.append("%s:%d: defines %s, which %s also defines: include its header or copy it "
                              "in a verbatim block" % (rel, line, name, src_constants[name]))

        code = blank(text)
        defined = {}
        for m in TEST_DEF.finditer(code):
            defined.setdefault(m.group(1), text.count("\n", 0, m.start()) + 1)
        run = set(RUN_TEST_CALL.findall(code))
        for name, line in sorted(defined.items(), key=lambda x: x[1]):
            if name not in run:
                errors.append("%s:%d: %s is never passed to RUN_TEST in this file" % (rel, line, name))

    # The anchored regions, hashed and compared with the lock (or written to it).
    regions = {}  # (src/file, spec): (first, last, sha256)
    unresolved = 0
    for (file, spec), users in sorted(anchors.items()):
        src = (root / file).read_bytes().decode().replace("\r\n", "\n")
        region = anchor_region(src, spec)
        if isinstance(region, str):
            unresolved += 1
            errors.append("%s: the anchor '%s' does not resolve in %s: %s" % (", ".join(users), spec, file, region))
        else:
            regions[(file, spec)] = region + (region_hash(src, *region),)
    lock_path = root / LOCK
    if relock:
        if unresolved:
            errors.append("--relock: %s not written, an anchor does not resolve" % LOCK)
        else:
            lock_path.write_text(LOCK_HEADER + "".join("%s %s %s\n" % (digest, file, spec)
                                                       for (file, spec), (_, _, digest) in sorted(regions.items())))
            print("wrote %s: %d anchored regions" % (LOCK, len(regions)))
    else:
        lock = read_lock(lock_path)
        for (file, spec), (first, last, digest) in sorted(regions.items()):
            if lock.get((file, spec)) != digest:
                errors.append("%s:%d-%d (%s) %s: review %s, then run %s"
                              % (file, first + 1, last + 1, spec,
                                 "changed since it was locked" if (file, spec) in lock else "is not in " + LOCK,
                                 ", ".join(anchors[(file, spec)]), RELOCK))
        for file, spec in sorted(set(lock) - set(anchors)):
            errors.append("%s: '%s %s' is anchored by no test: run %s" % (LOCK, file, spec, RELOCK))

    for e in errors:
        print("ERROR  " + e)
    for where, file, specs in sorted(adapted, key=lambda x: not x[2]):
        if not specs:
            print("not checked (adapted copy)  %s (%s)" % (where, file))
            continue
        spans = []
        for spec in specs:
            region = regions.get(("src/" + file, spec))
            spans.append("%s:%d-%d %s" % (file, region[0] + 1, region[1] + 1, spec) if region
                         else "%s %s" % (file, spec))
        print("locked (adapted copy)  %s (%s)" % (where, "; ".join(spans)))
    locked = sum(1 for _, _, specs in adapted if specs)
    print("%d verbatim cop%s checked, %d adapted cop%s listed (%d locked to %d src/ region%s, %d unlocked), %d error%s"
          % (checked, "y" if checked == 1 else "ies", len(adapted), "y" if len(adapted) == 1 else "ies",
             locked, len(anchors), "" if len(anchors) == 1 else "s", len(adapted) - locked,
             len(errors), "" if len(errors) == 1 else "s"))
    sys.exit(1 if errors or checked == 0 else 0)


if __name__ == "__main__":
    main()
