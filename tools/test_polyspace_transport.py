#!/usr/bin/env python3
"""Ensure CI never sends Polyspace Access credentials over plaintext HTTP."""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
POLYSPACE_JOB = ROOT / "ci" / "polyspace.sh"


def main() -> None:
    script = POLYSPACE_JOB.read_text(encoding="utf-8")
    assert re.search(r"-protocol\s+http(?:\s|\\\\|$)", script) is None
    assert script.count("-protocol https") == 2
    print("Polyspace Access transport: HTTPS-only")


if __name__ == "__main__":
    main()
