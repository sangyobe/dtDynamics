#!/usr/bin/env python3
"""Fail closed when an installed guide PDF drifts from its render inputs."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Mapping

from guide_provenance import discover_local_guide_assets


EXPECTED_GUIDES: tuple[dict[str, str], ...] = (
    {
        "label": "model-guide",
        "directory": "guides/model_guide",
        "manifest": "mjcf-xml-dtdynspec-dtdynmodel-guide.provenance.json",
        "source": "mjcf-xml-dtdynspec-dtdynmodel-guide.md",
        "pdf": "mjcf-xml-dtdynspec-dtdynmodel-guide.pdf",
    },
    {
        "label": "usage-guide",
        "directory": "guides/usage_guide",
        "manifest": "dtdynamics-usage-migration-guide.provenance.json",
        "source": "dtdynamics-usage-migration-guide.md",
        "pdf": "dtdynamics-usage-migration-guide.pdf",
    },
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_guide(docs_root: Path, guide: Mapping[str, str]) -> list[str]:
    label = str(guide["label"])
    guide_dir = docs_root / str(guide["directory"])
    manifest_path = guide_dir / str(guide["manifest"])
    source_path = guide_dir / str(guide["source"])
    pdf_path = guide_dir / str(guide["pdf"])
    theme_path = docs_root / "guides/themes/hmg.css"
    errors: list[str] = []

    for name, path in (("manifest", manifest_path), ("source", source_path),
                       ("theme", theme_path), ("PDF", pdf_path)):
        if not path.is_file():
            errors.append(f"{label}: missing installed {name}: {path.relative_to(docs_root)}")
    if errors:
        return errors

    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        return [f"{label}: invalid provenance JSON: {error}"]
    if not isinstance(manifest, dict):
        return [f"{label}: provenance manifest must be an object"]

    expected_values = {
        "schema": 2,
        "renderer": "@marp-team/marp-cli@4.5.1",
        "browser": "google-chrome-stable",
        "source": str(guide["source"]),
        "theme": "../themes/hmg.css",
        "pdf": str(guide["pdf"]),
    }
    for field, expected in expected_values.items():
        if manifest.get(field) != expected:
            errors.append(f"{label}: manifest field {field!r} is not {expected!r}")

    for field, path in (("source_sha256", source_path), ("theme_sha256", theme_path),
                        ("pdf_sha256", pdf_path)):
        if manifest.get(field) != sha256(path):
            errors.append(f"{label}: hash mismatch for {path.relative_to(docs_root)}")

    manifest_assets = manifest.get("assets")
    if not isinstance(manifest_assets, dict):
        return errors + [f"{label}: manifest assets must be an object"]
    expected_assets, discovery_errors = discover_local_guide_assets(source_path, docs_root)
    errors.extend(f"{label}: {error}" for error in discovery_errors)
    if set(manifest_assets) != set(expected_assets):
        errors.append(f"{label}: manifest asset set is incomplete or unexpected")
    for relative, path in expected_assets.items():
        if not path.is_file():
            continue
        if manifest_assets.get(relative) != sha256(path):
            errors.append(f"{label}: hash mismatch for {path.relative_to(docs_root)}")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--docs-root", required=True,
                        help="installed dtDynamics documentation root")
    args = parser.parse_args()

    docs_root = Path(args.docs_root).resolve()
    if not docs_root.is_dir():
        print(f"ERROR: installed docs root does not exist: {docs_root}", file=sys.stderr)
        return 1

    failures: list[str] = []
    for guide in EXPECTED_GUIDES:
        failures.extend(validate_guide(docs_root, guide))
    if failures:
        for failure in failures:
            print(f"ERROR: {failure}", file=sys.stderr)
        return 1

    print("installed guide provenance: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
