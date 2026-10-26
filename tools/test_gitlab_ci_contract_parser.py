#!/usr/bin/env python3
"""Legacy CI entry point for project-owned lint regression verification."""
from __future__ import annotations

import pathlib
import subprocess
import sys


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
LINT_SCRIPT = REPO_ROOT / "ci" / "lint.sh"
CONTRACT_TEST = REPO_ROOT / "tools" / "test_gitlab_ci_contract.py"


def main() -> int:
    syntax = subprocess.run(["bash", "-n", str(LINT_SCRIPT)], check=False)
    if syntax.returncode != 0:
        return syntax.returncode
    result = subprocess.run(
        [sys.executable, str(CONTRACT_TEST)],
        cwd=REPO_ROOT,
        check=False,
    )
    if result.returncode != 0:
        return result.returncode
    print("Project-owned lint contract entrypoint: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
