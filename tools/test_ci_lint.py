#!/usr/bin/env python3
"""Regression tests for the side-effect-free CI clang-format gate."""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
LINT = ROOT / "ci" / "lint.sh"


def make_fake_formatter(directory: Path) -> Path:
    formatter = directory / "clang-format"
    formatter.write_text(
        "#!/usr/bin/env sh\n"
        "if [ \"${FAKE_CLANG_FORMAT_EXIT:-0}\" -ne 0 ]; then\n"
        "  echo 'fake clang-format violation' >&2\n"
        "  exit \"${FAKE_CLANG_FORMAT_EXIT}\"\n"
        "fi\n"
        "exit 0\n",
        encoding="utf-8",
    )
    formatter.chmod(0o755)
    return formatter


def make_fake_package_test_python(directory: Path) -> Path:
    python = directory / "python3"
    python.write_text(
        "#!/usr/bin/env sh\n"
        "echo 'fake package-config test failure' >&2\n"
        "exit \"${FAKE_PACKAGE_TEST_EXIT}\"\n",
        encoding="utf-8",
    )
    python.chmod(0o755)
    return python


def make_fake_tee(directory: Path) -> Path:
    tee = directory / "tee"
    tee.write_text(
        "#!/usr/bin/env sh\n"
        "cat >/dev/null\n"
        "exit \"${FAKE_TEE_EXIT}\"\n",
        encoding="utf-8",
    )
    tee.chmod(0o755)
    return tee


def make_fake_find(directory: Path) -> Path:
    find = directory / "find"
    find.write_text(
        "#!/usr/bin/env sh\n"
        "printf 'include/sample.h\\0'\n"
        "exit \"${FAKE_FIND_EXIT}\"\n",
        encoding="utf-8",
    )
    find.chmod(0o755)
    return find


def run_lint(
    formatter_exit: int,
    tee_exit: int = 0,
    find_exit: int = 0,
    package_test_exit: int | None = None,
) -> tuple[subprocess.CompletedProcess[str], Path]:
    temporary = tempfile.TemporaryDirectory()
    root = Path(temporary.name)
    (root / "include").mkdir()
    (root / "src").mkdir()
    (root / "include" / "sample.h").write_text("int sample();\n", encoding="utf-8")
    (root / "src" / "sample.cpp").write_text("int sample() { return 1; }\n", encoding="utf-8")
    bin_directory = root / "bin"
    bin_directory.mkdir()
    make_fake_formatter(bin_directory)
    if package_test_exit is not None:
        make_fake_package_test_python(bin_directory)
    if tee_exit:
        make_fake_tee(bin_directory)
    if find_exit:
        make_fake_find(bin_directory)
    environment = {
        **os.environ,
        "PATH": f"{bin_directory}:{os.environ['PATH']}",
        "FAKE_CLANG_FORMAT_EXIT": str(formatter_exit),
        "FAKE_TEE_EXIT": str(tee_exit),
        "FAKE_FIND_EXIT": str(find_exit),
        "FAKE_PACKAGE_TEST_EXIT": str(package_test_exit or 0),
        "GIT_PUSH_TOKEN": "must-not-be-used",
    }
    result = subprocess.run(
        ["bash", str(LINT)],
        cwd=root,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    report = root / "reports" / "clang-format.txt"
    # Keep the temporary directory alive for callers that inspect the report.
    result._lint_temporary_directory = temporary  # type: ignore[attr-defined]
    return result, report


def assert_clean_format_passes_without_git_or_token() -> None:
    result, report = run_lint(0)
    try:
        assert result.returncode == 0, result.stdout + result.stderr
        assert report.exists()
        assert "dtDynamics config include precedence: PASS" in result.stdout
    finally:
        result._lint_temporary_directory.cleanup()  # type: ignore[attr-defined]


def assert_package_config_failure_blocks_lint() -> None:
    result, report = run_lint(0, package_test_exit=37)
    try:
        assert result.returncode == 37, result.stdout + result.stderr
        assert report.exists()
        assert "fake package-config test failure" in result.stderr
    finally:
        result._lint_temporary_directory.cleanup()  # type: ignore[attr-defined]


def assert_format_violation_fails_and_writes_report() -> None:
    result, report = run_lint(1)
    try:
        assert result.returncode == 1, result.stdout + result.stderr
        assert "fake clang-format violation" in report.read_text(encoding="utf-8")
    finally:
        result._lint_temporary_directory.cleanup()  # type: ignore[attr-defined]


def assert_report_write_failure_blocks_lint() -> None:
    result, report = run_lint(0, tee_exit=23)
    try:
        assert result.returncode != 0, result.stdout + result.stderr
        assert not report.exists()
    finally:
        result._lint_temporary_directory.cleanup()  # type: ignore[attr-defined]


def assert_source_discovery_failure_blocks_lint() -> None:
    result, report = run_lint(0, find_exit=31)
    try:
        assert result.returncode != 0, result.stdout + result.stderr
        assert "ERROR: clang-format source discovery failed" in result.stdout
        assert not report.exists()
    finally:
        result._lint_temporary_directory.cleanup()  # type: ignore[attr-defined]


def assert_lint_script_never_pushes_or_reads_write_token() -> None:
    script = LINT.read_text(encoding="utf-8")
    assert "GIT_PUSH_TOKEN" not in script
    assert "git push" not in script
    assert "PRIVATE-TOKEN" not in script


def main() -> None:
    assert_clean_format_passes_without_git_or_token()
    assert_package_config_failure_blocks_lint()
    assert_format_violation_fails_and_writes_report()
    assert_report_write_failure_blocks_lint()
    assert_source_discovery_failure_blocks_lint()
    assert_lint_script_never_pushes_or_reads_write_token()
    print("CI lint tests: PASS")


if __name__ == "__main__":
    main()
