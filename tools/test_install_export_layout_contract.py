#!/usr/bin/env python3
"""Lock selected lib/lib64 provenance for exported dtDynamics targets."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
VALIDATOR = ROOT / "tools" / "validate_install_export.sh"


def main() -> None:
    script = VALIDATOR.read_text(encoding="utf-8")

    required = (
        "DTDYNAMICS_EXPECTED_LIBRARY_DIR",
        'file(REAL_PATH "${DTDYNAMICS_EXPECTED_LIBRARY_DIR}" _DTDYNAMICS_EXPECTED_LIBRARY_DIR)',
        "function(_dtdynamics_require_within_selected_library_dir path label)",
        '"${_DTDYNAMICS_IMPORTED_LOCATION}" "dt::Dynamics imported location"',
        '-DDTDYNAMICS_EXPECTED_LIBRARY_DIR="${INSTALL_PREFIX}/${INSTALL_LIBDIR}"',
        '-DDTDYNAMICS_EXPECTED_LIBRARY_DIR="${RELOCATED_PREFIX}/${INSTALL_LIBDIR}"',
    )
    for token in required:
        assert token in script, f"missing selected install-layout provenance contract: {token}"

    print("install/export selected-layout provenance contract: PASS")


if __name__ == "__main__":
    main()
