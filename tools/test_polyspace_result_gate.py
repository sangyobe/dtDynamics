#!/usr/bin/env python3
"""Regression tests for the CI Polyspace result gate."""

from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
GATE = ROOT / "tools" / "polyspace_result_gate.py"
HEADER = (
    "ID\tFamily\tGroup\tColor\tNew\tCheck\tInformation\tClass\tFunction"
    "\tFile\tStatus\tSeverity\tComment\tKey\tLine\tCol\n"
)


def result_row(
    check: str,
    color: str,
    status: str,
    comment: str = "",
    function: str = "function",
    file: str = "src/example.cpp",
    line: int = 10,
) -> str:
    return (
        f"1\tDefect\tProgramming\t{color}\tno\t{check}\tImpact: Medium\tscope\t"
        f"{function}\t{file}\t{status}\tUnset\t{comment}\tkey\t{line}\t1\n"
    )


def run_gate(rows: list[str]) -> subprocess.CompletedProcess[str]:
    with tempfile.TemporaryDirectory() as directory:
        report = Path(directory) / "result_list.txt"
        report.write_text(HEADER + "".join(rows), encoding="utf-8")
        return subprocess.run(
            [sys.executable, str(GATE), "--input", str(report)],
            check=False,
            capture_output=True,
            text=True,
        )


def assert_gate_fails_for_unreviewed_red() -> None:
    result = run_gate([result_row("NULL_PTR", "Red", "Unreviewed")])
    assert result.returncode == 1, result.stdout + result.stderr
    assert "unresolved=1" in result.stdout
    assert "NULL_PTR" in result.stdout


def assert_gate_accepts_only_approved_accessor_justifications() -> None:
    locations = (
        ("dt::Dynamics::dtDynSpec::Bodies()", 1280),
        ("dt::Dynamics::dtDynSpec::Joints()", 1285),
        ("dt::Dynamics::dtDynSpec::Sites()", 1290),
        ("dt::Dynamics::dtDynSpec::Actuators()", 1295),
    )
    result = run_gate(
        [
            *[
                result_row(
                    "BREAKING_DATA_ENCAPSULATION",
                    "Red",
                    "Justified",
                    function=function,
                    file="/builds/dtdynamics/src/dtDynSpec.cpp",
                    line=line,
                )
                for function, line in locations
            ],
            result_row("NULL_PTR", "Green", "Unreviewed"),
        ]
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "unresolved=0" in result.stdout
    assert "approved_justified=4" in result.stdout


def assert_gate_rejects_unapproved_justification() -> None:
    result = run_gate([result_row("MISSING_CONSTEXPR", "Red", "No action planned")])
    assert result.returncode == 1, result.stdout + result.stderr
    assert "unresolved=1" in result.stdout


def assert_gate_rejects_extra_finding_at_approved_accessor() -> None:
    result = run_gate(
        [
            result_row(
                "BREAKING_DATA_ENCAPSULATION",
                "Red",
                "Justified",
                function="dt::Dynamics::dtDynSpec::Bodies()",
                file="/builds/dtdynamics/src/dtDynSpec.cpp",
                line=1281,
            )
        ]
    )
    assert result.returncode == 1, result.stdout + result.stderr
    assert "unresolved=1" in result.stdout


def assert_gate_rejects_nonjustified_approved_location() -> None:
    result = run_gate(
        [
            result_row(
                "BREAKING_DATA_ENCAPSULATION",
                "Red",
                "No action planned",
                function="dt::Dynamics::dtDynSpec::Bodies()",
                file="/builds/dtdynamics/src/dtDynSpec.cpp",
                line=1280,
            )
        ]
    )
    assert result.returncode == 1, result.stdout + result.stderr
    assert "unresolved=1" in result.stdout


def assert_gate_rejects_duplicate_approved_location() -> None:
    row = result_row(
        "BREAKING_DATA_ENCAPSULATION",
        "Red",
        "Justified",
        function="dt::Dynamics::dtDynSpec::Bodies()",
        file="/builds/dtdynamics/src/dtDynSpec.cpp",
        line=1280,
    )
    result = run_gate([row, row])
    assert result.returncode == 1, result.stdout + result.stderr
    assert "unresolved=1" in result.stdout


def assert_gate_fails_for_noncompiling_row() -> None:
    result = run_gate([result_row("File does not compile", "Red", "Unreviewed")])
    assert result.returncode == 1, result.stdout + result.stderr
    assert "unresolved=1" in result.stdout
    assert "noncompiling=1" in result.stdout


def assert_gate_fails_for_unreviewed_non_green_rows() -> None:
    result = run_gate(
        [
            result_row("UNCAUGHT_EXCEPTION", "Orange", "Unreviewed"),
            result_row("NULL_PTR", "Gray", "Unreviewed"),
            result_row("NULL_PTR", "Red", "Unreviewed", "File does not compile is only a comment."),
        ]
    )
    assert result.returncode == 1, result.stdout + result.stderr
    assert "unresolved=3" in result.stdout


def main() -> None:
    assert_gate_fails_for_unreviewed_red()
    assert_gate_accepts_only_approved_accessor_justifications()
    assert_gate_rejects_unapproved_justification()
    assert_gate_rejects_extra_finding_at_approved_accessor()
    assert_gate_rejects_nonjustified_approved_location()
    assert_gate_rejects_duplicate_approved_location()
    assert_gate_fails_for_noncompiling_row()
    assert_gate_fails_for_unreviewed_non_green_rows()
    print("polyspace result gate tests: PASS")


if __name__ == "__main__":
    main()
