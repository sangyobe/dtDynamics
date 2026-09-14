#!/usr/bin/env python3
"""Hash the complete uncommitted dtDynamics release-candidate surface.

Selection:
  1. every path reported by ``git status --porcelain=v1 -z``;
  2. the intentionally ignored private Marp sources and theme, when present.

Serialization is deterministic. For each unique path in UTF-8 bytewise-sorted
order, append:

    path + NUL + sha256(file-bytes) + NUL + decimal-byte-size + LF

Deleted paths, if any, use digest ``DELETED`` and size ``0``. Rename/copy
status entries are rejected so the candidate cannot be hashed ambiguously.
The reported candidate digest is SHA-256 of the concatenated serialization.
"""

from __future__ import annotations

import hashlib
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
PRIVATE_PATHS = (
    "docs/guides/model_guide/mjcf-xml-dtdynspec-dtdynmodel-guide.md",
    "docs/guides/usage_guide/dtdynamics-usage-migration-guide.md",
    "docs/guides/themes/hmg.css",
)


def git_output(*args: str) -> bytes:
    return subprocess.run(
        ["git", *args], cwd=ROOT, check=True, stdout=subprocess.PIPE
    ).stdout


def candidate_paths() -> list[str]:
    records = git_output("status", "--porcelain=v1", "-z").split(b"\0")
    paths: list[str] = []
    for record in records:
        if not record:
            continue
        status = record[:2].decode("ascii", errors="strict")
        if "R" in status or "C" in status:
            raise SystemExit(
                "Rename/copy status entries must be resolved before manifest hashing"
            )
        paths.append(record[3:].decode("utf-8", errors="strict"))

    for path in PRIVATE_PATHS:
        if (ROOT / path).is_file():
            paths.append(path)
    return sorted(set(paths), key=lambda value: value.encode("utf-8"))


def main() -> None:
    serialization = bytearray()
    paths = candidate_paths()
    for path in paths:
        file_path = ROOT / path
        if file_path.is_file():
            payload = file_path.read_bytes()
            digest = hashlib.sha256(payload).hexdigest()
            size = len(payload)
        elif not file_path.exists():
            digest = "DELETED"
            size = 0
        else:
            raise SystemExit(f"Candidate path is not a regular file: {path}")
        serialization.extend(
            path.encode("utf-8")
            + b"\0"
            + digest.encode("ascii")
            + b"\0"
            + str(size).encode("ascii")
            + b"\n"
        )

    print(f"candidate_files {len(paths)}")
    print(f"candidate_manifest_sha256 {hashlib.sha256(serialization).hexdigest()}")
    print("manifest_serialization utf8-path\\0sha256-or-DELETED\\0decimal-size\\n")


if __name__ == "__main__":
    main()
