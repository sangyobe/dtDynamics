#!/usr/bin/env python3
"""Ensure release-contract regressions are invoked by audit and GitLab CI."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
AUDIT = ROOT / "tools" / "audit_release_readiness.sh"
CI = ROOT / ".gitlab-ci.yml"


def main() -> None:
    audit = AUDIT.read_text(encoding="utf-8")
    ci = CI.read_text(encoding="utf-8")

    assert "require_file tools/test_package_release_contract.py" in audit
    assert 'python3 "${SCRIPT_DIR}/test_package_release_contract.py"' in audit
    for command in (
        'require_file tools/validate_installed_guide_provenance.py',
        'require_file tools/guide_provenance.py',
        'require_file tools/test_guide_provenance_parser.py',
        'require_file tools/test_installed_guide_provenance.py',
        'require_file tools/test_guide_provenance_reference_contract.py',
        'require_file tools/test_guide_install_contract.py',
        'require_file tools/install_validation_deps.sh',
        'require_file tools/test_validation_deps_installer.py',
        'require_file tools/test_dtdynamics_config_include_order.py',
        'require_file tools/test_ci_dtmath_dependency.py',
        'require_file tools/polyspace_result_gate.py',
        'require_file tools/test_polyspace_result_gate.py',
        'require_file tools/test_polyspace_export_contract.py',
        'require_file tools/test_polyspace_transport.py',
        'require_file tools/test_polyspace_checker_profile.py',
        'require_file tools/test_ci_lint.py',
        'python3 "${SCRIPT_DIR}/test_guide_provenance_parser.py"',
        'python3 "${SCRIPT_DIR}/test_installed_guide_provenance.py"',
        'python3 "${SCRIPT_DIR}/test_guide_provenance_reference_contract.py"',
        'python3 "${SCRIPT_DIR}/test_guide_install_contract.py"',
        'python3 "${SCRIPT_DIR}/test_validation_deps_installer.py"',
        'python3 "${SCRIPT_DIR}/test_dtdynamics_config_include_order.py"',
        'python3 "${SCRIPT_DIR}/test_ci_dtmath_dependency.py"',
        'python3 "${SCRIPT_DIR}/test_polyspace_result_gate.py"',
        'python3 "${SCRIPT_DIR}/test_polyspace_export_contract.py"',
        'python3 "${SCRIPT_DIR}/test_polyspace_transport.py"',
        'python3 "${SCRIPT_DIR}/test_polyspace_checker_profile.py"',
        'python3 "${SCRIPT_DIR}/test_ci_lint.py"',
    ):
        assert command in audit, f"release audit is missing: {command}"

    assert "release-contracts:" in ci
    assert "package-validation:" in ci
    package_job = ci.split("package-validation:", 1)[1].split("\n\n", 1)[0]
    assert "stage: build" in package_job
    assert "./tools/validate_package.sh --skip-reference-tests" in package_job
    assert "allow_failure:" not in package_job
    for command in (
        "python3 tools/test_package_release_contract.py",
        "python3 tools/test_install_export_layout_contract.py",
        "python3 tools/test_installed_guide_provenance.py",
        "python3 tools/test_guide_install_contract.py",
        "python3 tools/test_validation_deps_installer.py",
        "python3 tools/test_dtdynamics_config_include_order.py",
        "python3 tools/test_ci_dtmath_dependency.py",
        "python3 tools/test_polyspace_result_gate.py",
        "python3 tools/test_polyspace_export_contract.py",
        "python3 tools/test_polyspace_transport.py",
        "python3 tools/test_polyspace_checker_profile.py",
    ):
        assert command in ci, f"GitLab release-contracts job is missing: {command}"

    markdown_docs = ci.split("markdown-docs:", 1)[1]
    assert "python3 tools/test_guide_provenance_parser.py" in markdown_docs, (
        "GitLab markdown-docs job is missing guide-provenance syntax coverage"
    )
    assert "python3 tools/test_guide_provenance_reference_contract.py" in markdown_docs, (
        "GitLab markdown-docs job is missing source guide-provenance reference coverage"
    )

    print("release-contract gate wiring: PASS")


if __name__ == "__main__":
    main()
