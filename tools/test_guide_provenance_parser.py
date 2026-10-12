#!/usr/bin/env python3
"""Regression tests for syntax-aware guide image-reference discovery."""

from __future__ import annotations

from pathlib import Path
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from guide_provenance import discover_local_guide_assets  # noqa: E402


def write_source(root: Path, content: str) -> tuple[Path, Path]:
    docs_root = root / "docs"
    guide_dir = docs_root / "guides/model_guide"
    guide_dir.mkdir(parents=True)
    source_path = guide_dir / "guide.md"
    source_path.write_text(content, encoding="utf-8")
    return docs_root, guide_dir


def assert_reference_style_image_is_discovered() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        docs_root, guide_dir = write_source(
            Path(temporary),
            "![runtime lifecycle][lifecycle]\n\n"
            "[lifecycle]: figures/untracked.svg \"render input\"\n",
        )
        asset = guide_dir / "figures/untracked.svg"
        asset.parent.mkdir(parents=True)
        asset.write_text("<svg/>", encoding="utf-8")
        assets, errors = discover_local_guide_assets(guide_dir / "guide.md", docs_root)
        assert errors == [], errors
        assert assets == {"figures/untracked.svg": asset.resolve()}, assets


def assert_html_image_with_quoted_gt_is_discovered() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        docs_root, guide_dir = write_source(
            Path(temporary),
            '<img alt="runtime 1 > 0" src="figures/untracked.svg" />\n',
        )
        asset = guide_dir / "figures/untracked.svg"
        asset.parent.mkdir(parents=True)
        asset.write_text("<svg/>", encoding="utf-8")
        assets, errors = discover_local_guide_assets(guide_dir / "guide.md", docs_root)
        assert errors == [], errors
        assert assets == {"figures/untracked.svg": asset.resolve()}, assets


def assert_html_remote_image_with_quoted_gt_is_rejected() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        docs_root, guide_dir = write_source(
            Path(temporary),
            '<img alt="runtime 1 > 0" src="https://example.invalid/untracked.svg" />\n',
        )
        assets, errors = discover_local_guide_assets(guide_dir / "guide.md", docs_root)
        assert assets == {}, assets
        assert any("must be a local path" in error for error in errors), errors


def assert_non_img_visual_html_is_rejected() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        docs_root, guide_dir = write_source(
            Path(temporary),
            "<picture><source srcset=\"figures/unbound.svg\">"
            "<img src=\"figures/fallback.svg\"></picture>\n"
            "<svg><image href=\"figures/unbound.svg\" /></svg>\n",
        )
        for name in ("fallback.svg", "unbound.svg"):
            asset = guide_dir / "figures" / name
            asset.parent.mkdir(parents=True, exist_ok=True)
            asset.write_text("<svg/>", encoding="utf-8")
        _, errors = discover_local_guide_assets(guide_dir / "guide.md", docs_root)
        assert any("<picture>" in error for error in errors), errors
        assert any("<svg>" in error for error in errors), errors


def assert_masked_markdown_image_syntax_is_rejected() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        docs_root, guide_dir = write_source(
            Path(temporary),
            "\\`![unbound](figures/unbound.svg)`\n"
            "``` `\n"
            "![unbound](figures/unbound.svg)\n"
            "```\n",
        )
        asset = guide_dir / "figures/unbound.svg"
        asset.parent.mkdir(parents=True)
        asset.write_text("<svg/>", encoding="utf-8")
        _, errors = discover_local_guide_assets(guide_dir / "guide.md", docs_root)
        assert any("masked Markdown" in error for error in errors), errors


def main() -> int:
    assert_reference_style_image_is_discovered()
    assert_html_image_with_quoted_gt_is_discovered()
    assert_html_remote_image_with_quoted_gt_is_rejected()
    assert_non_img_visual_html_is_rejected()
    assert_masked_markdown_image_syntax_is_rejected()
    print("guide provenance parser tests: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
