#!/usr/bin/env python3
"""Regression test for a failed Polyspace export with a stale prior report."""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
POLYSPACE_JOB = ROOT / "ci" / "polyspace.sh"
GATE = ROOT / "tools" / "polyspace_result_gate.py"
HEADER = (
    "ID\tFamily\tGroup\tColor\tNew\tCheck\tInformation\tClass\tFunction"
    "\tFile\tStatus\tSeverity\tComment\tKey\tLine\tCol\n"
)
GREEN_ROW = (
    "1\tDefect\tProgramming\tGreen\tno\tNULL_PTR\tImpact: Medium\tscope"
    "\tfunction\tsrc/example.cpp\tUnreviewed\tUnset\t\tkey\t10\t1\n"
)


def write_executable(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")
    path.chmod(0o755)


def run_job(
    analyzer_exit: int = 0,
    exporter_exit: int = 9,
    configure_exit: int = 0,
    configure_writes_options: bool = True,
    cleanup_exit: int = 0,
    report_content: str = HEADER + GREEN_ROW,
    exporter_writes_report: bool = True,
    report_generator_exit: int = 0,
    server_version_exit: int = 0,
) -> subprocess.CompletedProcess[str]:
    temporary = tempfile.TemporaryDirectory()
    root = Path(temporary.name)
    (root / "bin").mkdir()
    (root / "build").mkdir()
    (root / "build-polyspace").mkdir()
    (root / "polyspace").mkdir()
    (root / "tools").mkdir()
    (root / "reports" / "polyspace" / "polyspace").mkdir(parents=True)
    (root / "build" / "compile_commands.json").write_text("[]\n", encoding="utf-8")
    (root / "build-polyspace" / "configure.opts").write_text("stale\n", encoding="utf-8")
    (root / "polyspace" / "polyspace.opts").write_text("# fixture\n", encoding="utf-8")
    (root / "polyspace" / "checkers.xml").write_text("<fixture/>\n", encoding="utf-8")
    (root / "polyspace" / "setup_phase_deviation.txt").write_text(
        "21.6.1 src/example_setup.cpp\n", encoding="utf-8"
    )
    report_template = root / "green-result-list.txt"
    report_template.write_text(report_content, encoding="utf-8")
    (root / "reports" / "polyspace" / "polyspace" / "result_list.txt").write_text(
        HEADER + GREEN_ROW,
        encoding="utf-8",
    )
    (root / "reports" / "polyspace" / "polyspace-report.pdf").write_text(
        "stale PDF artifact\n",
        encoding="utf-8",
    )
    shutil.copy2(GATE, root / "tools" / "polyspace_result_gate.py")

    if cleanup_exit:
        write_executable(
            root / "bin" / "rm",
            "#!/usr/bin/env sh\n"
            "for argument in \"$@\"; do\n"
            "  if [ \"${argument}\" = \"reports/polyspace/results\" ]; then exit \"${FAKE_CLEANUP_EXIT}\"; fi\n"
            "done\n"
            "exec /bin/rm \"$@\"\n",
        )
    write_executable(
        root / "bin" / "polyspace-configure",
        "#!/usr/bin/env sh\n"
        "if [ \"${FAKE_CONFIGURE_EXIT}\" -ne 0 ]; then exit \"${FAKE_CONFIGURE_EXIT}\"; fi\n"
        "while [ $# -gt 0 ]; do\n"
        "  if [ \"$1\" = \"-output-options-file\" ]; then output=$2; shift 2; continue; fi\n"
        "  shift\n"
        "done\n"
        "if [ \"${FAKE_CONFIGURE_WRITES_OPTIONS}\" = 1 ]; then\n"
        "  mkdir -p \"$(dirname \"$output\")\"\n"
        "  printf 'fixture' > \"$output\"\n"
        "fi\n",
    )
    write_executable(
        root / "bin" / "polyspace-bug-finder-server",
        "#!/usr/bin/env sh\n"
        "if [ \"${1:-}\" = \"-version\" ]; then exit \"${FAKE_SERVER_VERSION_EXIT}\"; fi\n"
        "while [ $# -gt 0 ]; do\n"
        "  if [ \"$1\" = \"-results-dir\" ]; then results=$2; shift 2; continue; fi\n"
        "  shift\n"
        "done\n"
        "mkdir -p \"$results\"\n"
        "printf 'fixture' > \"$results/ps_results.psbf\"\n"
        "exit \"${FAKE_ANALYZER_EXIT}\"\n",
    )
    write_executable(
        root / "bin" / "polyspace-results-export",
        "#!/usr/bin/env sh\n"
        "if [ \"${FAKE_EXPORT_EXIT}\" -ne 0 ]; then exit \"${FAKE_EXPORT_EXIT}\"; fi\n"
        "while [ $# -gt 0 ]; do\n"
        "  if [ \"$1\" = \"-output-name\" ]; then output=$2; shift 2; continue; fi\n"
        "  shift\n"
        "done\n"
        "if [ \"${FAKE_EXPORT_WRITES_REPORT}\" = 1 ]; then\n"
        "  mkdir -p \"$output\"\n"
        "  cp \"${FAKE_REPORT_TEMPLATE}\" \"$output/result_list.txt\"\n"
        "fi\n",
    )
    write_executable(
        root / "bin" / "polyspace-report-generator",
        "#!/usr/bin/env sh\nexit \"${FAKE_REPORT_GENERATOR_EXIT}\"\n",
    )

    environment = {
        **os.environ,
        "PATH": f"{root / 'bin'}:{os.environ['PATH']}",
        "POLYSPACE_PROJECT_NAME": "dtdynamics",
        "POLYSPACE_ACCESS_PARENT": "art_ws/dtdynamics",
        "POLYSPACE_BIN": "/fake/polyspace/bin",
        "POLYSPACE_TEMPLATES": "/fake/polyspace/templates",
        "POLYSPACE_ACCESS_HOST": "127.0.0.1",
        "POLYSPACE_ACCESS_PORT": "9444",
        "CI_COMMIT_REF_SLUG": "ci-test",
        "CI_COMMIT_REF_NAME": "ci-test",
        "CI_COMMIT_SHA": "0123456789abcdef",
        "CI_COMMIT_SHORT_SHA": "01234567",
        "CI_PIPELINE_IID": "1",
        "FAKE_ANALYZER_EXIT": str(analyzer_exit),
        "FAKE_EXPORT_EXIT": str(exporter_exit),
        "FAKE_CONFIGURE_EXIT": str(configure_exit),
        "FAKE_CONFIGURE_WRITES_OPTIONS": "1" if configure_writes_options else "0",
        "FAKE_CLEANUP_EXIT": str(cleanup_exit),
        "FAKE_REPORT_TEMPLATE": str(report_template),
        "FAKE_EXPORT_WRITES_REPORT": "1" if exporter_writes_report else "0",
        "FAKE_REPORT_GENERATOR_EXIT": str(report_generator_exit),
        "FAKE_SERVER_VERSION_EXIT": str(server_version_exit),
    }
    result = subprocess.run(
        ["bash", str(POLYSPACE_JOB)],
        cwd=root,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    result._polyspace_temporary_directory = temporary  # type: ignore[attr-defined]
    return result


def assert_failed_export_cannot_reuse_stale_report() -> None:
    result = run_job(exporter_exit=9)
    try:
        assert result.returncode != 0, result.stdout + result.stderr
        assert "ERROR: Polyspace result export failed" in result.stdout
    finally:
        result._polyspace_temporary_directory.cleanup()  # type: ignore[attr-defined]


def assert_missing_exported_report_cannot_reuse_stale_report() -> None:
    result = run_job(exporter_exit=0, exporter_writes_report=False)
    try:
        assert result.returncode != 0, result.stdout + result.stderr
        assert "ERROR: Polyspace export did not produce canonical result_list.txt" in result.stdout
    finally:
        result._polyspace_temporary_directory.cleanup()  # type: ignore[attr-defined]


def assert_failed_pdf_generation_cannot_publish_stale_pdf() -> None:
    result = run_job(exporter_exit=0, report_generator_exit=7)
    try:
        assert result.returncode == 0, result.stdout + result.stderr
        stale_pdf = Path(result._polyspace_temporary_directory.name) / "reports/polyspace/polyspace-report.pdf"  # type: ignore[attr-defined]
        assert not stale_pdf.exists(), f"stale Polyspace PDF survived: {stale_pdf}"
    finally:
        result._polyspace_temporary_directory.cleanup()  # type: ignore[attr-defined]


def assert_failed_analysis_cannot_publish_partial_result() -> None:
    result = run_job(analyzer_exit=17, exporter_exit=0)
    try:
        assert result.returncode != 0, result.stdout + result.stderr
        assert "ERROR: Polyspace analysis command failed" in result.stdout
    finally:
        result._polyspace_temporary_directory.cleanup()  # type: ignore[attr-defined]


def assert_reviewed_green_row_fails_raw_zero_gate() -> None:
    reviewed_green = (HEADER + GREEN_ROW).replace("\tUnreviewed\t", "\tJustified\t")
    result = run_job(exporter_exit=0, report_content=reviewed_green)
    try:
        assert result.returncode != 0, result.stdout + result.stderr
        assert "Polyspace result gate: unresolved=1 justified=1 noncompiling=0" in result.stdout
    finally:
        result._polyspace_temporary_directory.cleanup()  # type: ignore[attr-defined]


def assert_truncated_schema_cannot_pass_the_ci_wrapper() -> None:
    truncated = (
        "Color\tCheck\tStatus\tFile\tLine\tFunction\n"
        "Green\tNULL_PTR\tUnreviewed\tsrc/example.cpp\t10\tfunction\n"
    )
    result = run_job(exporter_exit=0, report_content=truncated)
    try:
        assert result.returncode != 0, result.stdout + result.stderr
        assert "canonical TSV header/schema" in result.stderr
    finally:
        result._polyspace_temporary_directory.cleanup()  # type: ignore[attr-defined]


def assert_failed_configuration_cannot_reuse_stale_options() -> None:
    result = run_job(configure_exit=13, exporter_exit=0)
    try:
        assert result.returncode != 0, result.stdout + result.stderr
        assert "ERROR: Polyspace configuration failed" in result.stdout
    finally:
        result._polyspace_temporary_directory.cleanup()  # type: ignore[attr-defined]


def assert_empty_configuration_cannot_reuse_stale_options() -> None:
    result = run_job(configure_exit=0, configure_writes_options=False, exporter_exit=0)
    try:
        assert result.returncode != 0, result.stdout + result.stderr
        assert "ERROR: Polyspace configuration did not produce fresh options" in result.stdout
    finally:
        result._polyspace_temporary_directory.cleanup()  # type: ignore[attr-defined]


def assert_failed_cleanup_cannot_continue() -> None:
    result = run_job(cleanup_exit=29, exporter_exit=0)
    try:
        assert result.returncode != 0, result.stdout + result.stderr
        assert "ERROR: cannot clear prior Polyspace results" in result.stdout
    finally:
        result._polyspace_temporary_directory.cleanup()  # type: ignore[attr-defined]


def assert_server_preflight_failure_cannot_continue() -> None:
    result = run_job(server_version_exit=17, exporter_exit=0)
    try:
        assert result.returncode != 0, result.stdout + result.stderr
        assert "ERROR: Polyspace binary/license-server preflight failed" in result.stdout
        root = Path(result._polyspace_temporary_directory.name)  # type: ignore[attr-defined]
        assert not (root / "reports" / "polyspace" / "results").exists()
    finally:
        result._polyspace_temporary_directory.cleanup()  # type: ignore[attr-defined]


def main() -> None:
    assert_failed_export_cannot_reuse_stale_report()
    assert_missing_exported_report_cannot_reuse_stale_report()
    assert_failed_pdf_generation_cannot_publish_stale_pdf()
    assert_failed_analysis_cannot_publish_partial_result()
    assert_reviewed_green_row_fails_raw_zero_gate()
    assert_truncated_schema_cannot_pass_the_ci_wrapper()
    assert_failed_configuration_cannot_reuse_stale_options()
    assert_empty_configuration_cannot_reuse_stale_options()
    assert_failed_cleanup_cannot_continue()
    assert_server_preflight_failure_cannot_continue()
    print("Polyspace cleanup, configuration, export, and analysis contracts: PASS")


if __name__ == "__main__":
    main()
