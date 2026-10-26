#!/usr/bin/env python3
"""Regression coverage for source-guide asset discovery and provenance closure."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
VALIDATOR_REL = Path("tools/validate_documentation_content.py")
COPY_PATHS = (
    Path("README.md"),
    Path(".gitlab-ci.yml"),
    Path("ci"),
    Path("docs"),
    Path("include"),
    Path("src"),
    Path("benchmark"),
    Path("tools"),
)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def copy_validator_fixture(destination: Path) -> None:
    for relative in COPY_PATHS:
        source = ROOT / relative
        target = destination / relative
        if source.is_dir():
            shutil.copytree(source, target, ignore=shutil.ignore_patterns("__pycache__"))
        else:
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)


def run_validator(root: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(root / VALIDATOR_REL)],
        cwd=root,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def main() -> int:
    with tempfile.TemporaryDirectory() as temporary:
        fixture_root = Path(temporary) / "dtdynamics"
        copy_validator_fixture(fixture_root)

        source_path = fixture_root / "docs/guides/model_guide/mjcf-xml-dtdynspec-dtdynmodel-guide.md"
        asset_path = fixture_root / "docs/guides/model_guide/figures/probe-unbound.svg"
        asset_path.write_text("<svg xmlns=\"http://www.w3.org/2000/svg\"/>", encoding="utf-8")
        source_path.write_text(
            source_path.read_text(encoding="utf-8")
            + "\n![unbound render input][unbound]\n\n[unbound]: figures/probe-unbound.svg\n",
            encoding="utf-8",
        )

        manifest_path = source_path.with_suffix(".provenance.json")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["source_sha256"] = digest(source_path)
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

        result = run_validator(fixture_root)
        assert result.returncode != 0, result.stdout
        assert "model-guide PDF provenance manifest asset set is incomplete or unexpected" in result.stderr, result.stderr

    with tempfile.TemporaryDirectory() as temporary:
        fixture_root = Path(temporary) / "dtdynamics"
        copy_validator_fixture(fixture_root)
        source_path = fixture_root / "docs/guides/model_guide/mjcf-xml-dtdynspec-dtdynmodel-guide.md"
        source_path.write_text(
            source_path.read_text(encoding="utf-8")
            + '\n<img alt="runtime 1 > 0" src="https://example.invalid/unbound.svg" />\n',
            encoding="utf-8",
        )
        manifest_path = source_path.with_suffix(".provenance.json")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["source_sha256"] = digest(source_path)
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

        result = run_validator(fixture_root)
        assert result.returncode != 0, result.stdout
        assert "image reference must be a local path" in result.stderr, result.stderr

    with tempfile.TemporaryDirectory() as temporary:
        fixture_root = Path(temporary) / "dtdynamics"
        copy_validator_fixture(fixture_root)
        source_path = fixture_root / "docs/guides/model_guide/mjcf-xml-dtdynspec-dtdynmodel-guide.md"
        source_path.write_text(
            source_path.read_text(encoding="utf-8")
            + "\n<picture><source srcset=\"figures/unbound.svg\"><img src=\"figures/dtdynamics_to_be.svg\"></picture>\n"
            + "<svg><image href=\"figures/unbound.svg\" /></svg>\n"
            + "\\`![unbound](figures/unbound.svg)`\n",
            encoding="utf-8",
        )
        manifest_path = source_path.with_suffix(".provenance.json")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["source_sha256"] = digest(source_path)
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

        result = run_validator(fixture_root)
        assert result.returncode != 0, result.stdout
        assert "unsupported visual HTML tag <picture>" in result.stderr, result.stderr
        assert "unsupported visual HTML tag <svg>" in result.stderr, result.stderr
        assert "rendered image syntax inside masked Markdown is unsupported" in result.stderr, result.stderr

    print("source guide-provenance reference contract: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
