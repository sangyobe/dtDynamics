#!/usr/bin/env python3
"""Run the project-owned lint regression without parsing GitLab YAML."""
from __future__ import annotations

import pathlib
import subprocess
import sys


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
LINT_REGRESSION = REPO_ROOT / "tools" / "test_ci_lint.py"


def main() -> int:
    result = subprocess.run(
        [sys.executable, str(LINT_REGRESSION)],
        cwd=REPO_ROOT,
        check=False,
    )
    if result.returncode != 0:
        return result.returncode
    print("Project-owned lint package-config wiring: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
