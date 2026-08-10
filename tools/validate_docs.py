#!/usr/bin/env python3
"""Fail-closed GitLab Markdown documentation validation for dtDynamics."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
DOCS = [ROOT / "README.md", ROOT / "docs" / "manual.md", ROOT / "docs" / "benchmark-results.md"]
DOCS += sorted((ROOT / "docs" / "manual").glob("*.md"))
errors = []

def local_target(source: Path, raw: str):
    raw = raw.split("#", 1)[0].split("?", 1)[0].strip()
    if not raw or raw.startswith(("https://", "http://", "mailto:", "#")):
        return None
    return (source.parent / raw).resolve()

for path in DOCS:
    text = path.read_text(encoding="utf-8")
    rel = path.relative_to(ROOT)
    for legacy in ("@ref", "@dotfile", "\\f[", "\\f$"):
        if legacy in text:
            errors.append(f"retired documentation syntax {legacy!r} remains in {rel}")
    fence_count = sum(1 for line in text.splitlines() if line.startswith("```"))
    if fence_count % 2:
        errors.append(f"unclosed fenced block in {rel}")
    # GitLab inline math uses $`...`$. Shell code frequently contains a
    # backtick immediately before `$HOME`; avoid treating that as math. Count
    # only complete GitLab math spans here; unmatched delimiters remain visible
    # in Markdown review rather than producing false failures on shell syntax.
    _complete_math_spans = re.findall(r"\$`[^\n]*?`\$", text)
    for match in re.finditer(r"!?\[[^]]*\]\(([^)]+)\)", text):
        target = local_target(path, match.group(1))
        if target is not None and not target.exists():
            errors.append(f"broken local Markdown link in {rel}: {match.group(1)}")

diagrams = ROOT / "docs" / "manual" / "diagrams"
for dot in diagrams.glob("*.dot"):
    svg = dot.with_suffix(".svg")
    if not svg.is_file() or svg.stat().st_size == 0:
        errors.append(f"missing rendered SVG: {svg.relative_to(ROOT)}")

manual = (ROOT / "docs" / "manual.md").read_text(encoding="utf-8")
for token in ("CompileSpecChecked", "DTDYN_INVALID_INDEX", "dtDynData::Resize", "tools/validate_package.sh"):
    if token not in manual:
        errors.append(f"manual.md missing required how-to token: {token}")

if errors:
    print("FAIL: GitLab Markdown documentation validation")
    for error in errors:
        print("-", error)
    raise SystemExit(1)
print("PASS: GitLab Markdown manuals validated")
