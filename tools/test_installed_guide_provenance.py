#!/usr/bin/env python3
"""Regression coverage for installed guide-provenance validation."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
VALIDATOR = ROOT / "tools" / "validate_installed_guide_provenance.py"


MODEL_ASSETS = (
    "figures/mjcf_dtdyn_lifecycle.svg",
    "figures/dtdynamics_to_be.svg",
    "figures/runtime_model_update.svg",
)
USAGE_ASSETS = (
    "figures/dtdynamics_as_is.svg",
    "figures/dtdynamics_to_be.svg",
    "../../figures/runtime_by_fixture.png",
    "../../figures/scaling_by_dof.png",
    "../../figures/stage_breakdown.png",
    "../../figures/latency_cdf.png",
)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_artifact(root: Path, relative: str, content: bytes) -> Path:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)
    return path


def write_manifest(root: Path, guide: str, source: str, pdf: str, assets: tuple[str, ...]) -> None:
    guide_dir = root / "guides" / guide
    source_path = guide_dir / source
    theme_path = root / "guides/themes/hmg.css"
    pdf_path = guide_dir / pdf
    asset_hashes = {rel: digest((guide_dir / rel).resolve()) for rel in assets}
    manifest = {
        "schema": 2,
        "renderer": "@marp-team/marp-cli@4.5.1",
        "browser": "google-chrome-stable",
        "source": source,
        "source_sha256": digest(source_path),
        "theme": "../themes/hmg.css",
        "theme_sha256": digest(theme_path),
        "assets": asset_hashes,
        "pdf": pdf,
        "pdf_sha256": digest(pdf_path),
    }
    (guide_dir / f"{Path(source).stem}.provenance.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )


def refresh_manifest_source_hash(root: Path, guide: str, source: str) -> None:
    guide_dir = root / "guides" / guide
    manifest_path = guide_dir / f"{Path(source).stem}.provenance.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["source_sha256"] = digest(guide_dir / source)
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


def make_docs_root(root: Path) -> None:
    write_artifact(root, "guides/themes/hmg.css", b"theme")

    model_dir = root / "guides/model_guide"
    model_source = "\n".join(f"![model asset]({asset})" for asset in MODEL_ASSETS).encode()
    write_artifact(model_dir, "mjcf-xml-dtdynspec-dtdynmodel-guide.md", model_source)
    write_artifact(model_dir, "mjcf-xml-dtdynspec-dtdynmodel-guide.pdf", b"model-pdf")
    for asset in MODEL_ASSETS:
        write_artifact(model_dir, asset, f"model:{asset}".encode())
    write_manifest(root, "model_guide", "mjcf-xml-dtdynspec-dtdynmodel-guide.md",
                   "mjcf-xml-dtdynspec-dtdynmodel-guide.pdf", MODEL_ASSETS)

    usage_dir = root / "guides/usage_guide"
    usage_source = "\n".join(f'<img src="{asset}" />' for asset in USAGE_ASSETS).encode()
    write_artifact(usage_dir, "dtdynamics-usage-migration-guide.md", usage_source)
    write_artifact(usage_dir, "dtdynamics-usage-migration-guide.pdf", b"usage-pdf")
    for asset in USAGE_ASSETS:
        write_artifact(usage_dir, asset, f"usage:{asset}".encode())
    write_manifest(root, "usage_guide", "dtdynamics-usage-migration-guide.md",
                   "dtdynamics-usage-migration-guide.pdf", USAGE_ASSETS)


def run(root: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(VALIDATOR), "--docs-root", str(root)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def main() -> int:
    with tempfile.TemporaryDirectory() as temporary:
        docs_root = Path(temporary) / "docs"
        make_docs_root(docs_root)

        clean = run(docs_root)
        assert clean.returncode == 0, clean.stderr
        assert "installed guide provenance: PASS" in clean.stdout, clean.stdout

        stale_asset = docs_root / "guides/model_guide/figures/mjcf_dtdyn_lifecycle.svg"
        stale_asset.write_bytes(b"changed-after-render")
        stale = run(docs_root)
        assert stale.returncode != 0, stale.stdout
        assert "hash mismatch" in stale.stderr, stale.stderr

    with tempfile.TemporaryDirectory() as temporary:
        docs_root = Path(temporary) / "docs"
        make_docs_root(docs_root)
        model_dir = docs_root / "guides/model_guide"
        source_name = "mjcf-xml-dtdynspec-dtdynmodel-guide.md"
        write_artifact(model_dir, "figures/probe-unbound.svg", b"unbound-render-input")
        source_path = model_dir / source_name
        source_path.write_text(
            source_path.read_text(encoding="utf-8")
            + "\n![unbound render input][unbound]\n\n[unbound]: figures/probe-unbound.svg\n",
            encoding="utf-8",
        )
        refresh_manifest_source_hash(docs_root, "model_guide", source_name)
        unbound = run(docs_root)
        assert unbound.returncode != 0, unbound.stdout
        assert "asset set is incomplete or unexpected" in unbound.stderr, unbound.stderr

    with tempfile.TemporaryDirectory() as temporary:
        docs_root = Path(temporary) / "docs"
        make_docs_root(docs_root)
        usage_dir = docs_root / "guides/usage_guide"
        source_name = "dtdynamics-usage-migration-guide.md"
        source_path = usage_dir / source_name
        source_path.write_text(
            source_path.read_text(encoding="utf-8")
            + '\n<img alt="runtime 1 > 0" src="https://example.invalid/unbound.svg" />\n',
            encoding="utf-8",
        )
        refresh_manifest_source_hash(docs_root, "usage_guide", source_name)
        remote = run(docs_root)
        assert remote.returncode != 0, remote.stdout
        assert "image reference must be a local path" in remote.stderr, remote.stderr

    with tempfile.TemporaryDirectory() as temporary:
        docs_root = Path(temporary) / "docs"
        make_docs_root(docs_root)
        model_dir = docs_root / "guides/model_guide"
        source_name = "mjcf-xml-dtdynspec-dtdynmodel-guide.md"
        source_path = model_dir / source_name
        source_path.write_text(
            source_path.read_text(encoding="utf-8")
            + "\n<picture><source srcset=\"figures/unbound.svg\"><img src=\"figures/dtdynamics_to_be.svg\"></picture>\n"
            + "<svg><image href=\"figures/unbound.svg\" /></svg>\n"
            + "\\`![unbound](figures/unbound.svg)`\n",
            encoding="utf-8",
        )
        refresh_manifest_source_hash(docs_root, "model_guide", source_name)
        unsupported = run(docs_root)
        assert unsupported.returncode != 0, unsupported.stdout
        assert "unsupported visual HTML tag <picture>" in unsupported.stderr, unsupported.stderr
        assert "unsupported visual HTML tag <svg>" in unsupported.stderr, unsupported.stderr
        assert "rendered image syntax inside masked Markdown is unsupported" in unsupported.stderr, unsupported.stderr

    print("installed guide-provenance contract: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
