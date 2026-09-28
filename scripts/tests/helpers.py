"""Shared plumbing for the scripts/tests/ suite: no third-party dependency, run with
    python3 -m unittest discover -s scripts/tests
"""

import pathlib
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parents[2]
SCRIPTS = REPO / "scripts"


def run_script(name, args=(), cwd=None):
    """Runs scripts/<name> with args; returns (returncode, stdout, stderr)."""
    proc = subprocess.run(
        [sys.executable, str(SCRIPTS / name), *args],
        cwd=str(cwd) if cwd else None,
        capture_output=True,
        text=True,
    )
    return proc.returncode, proc.stdout, proc.stderr


def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
