#!/usr/bin/env python3
"""Static contract for installing guide sources and provenance inputs."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    installer = (ROOT / "tools/validate_install_export.sh").read_text(encoding="utf-8")

    forbidden = (
        'REGEX "/guides/themes(/|$)" EXCLUDE',
        'REGEX "/guides/(model_guide|usage_guide)/[^/]+\\\\.md$" EXCLUDE',
    )
    for token in forbidden:
        assert token not in cmake, f"CMake excludes guide provenance input: {token}"

    required = (
        'INSTALL_DOCDIR="${DTDYNAMICS_INSTALL_DOCDIR:-share/doc/dtDynamics}"',
        '-DCMAKE_INSTALL_DOCDIR="${INSTALL_DOCDIR}"',
        'DOC_DIR="${INSTALL_PREFIX}/${INSTALL_DOCDIR}"',
        'python3 "${SCRIPT_DIR}/validate_installed_guide_provenance.py" --docs-root "${DOC_DIR}"',
        'RELOCATED_DOC_DIR="${RELOCATED_PREFIX}/${INSTALL_DOCDIR}"',
        'python3 "${SCRIPT_DIR}/validate_installed_guide_provenance.py" --docs-root "${RELOCATED_DOC_DIR}"',
    )
    for token in required:
        assert token in installer, f"install/export validation does not prove guide provenance: {token}"

    print("guide install/export contract: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
