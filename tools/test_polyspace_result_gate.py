#!/usr/bin/env python3
"""Regression tests for the fail-closed Polyspace result gate."""

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


def run_raw_report(content: str) -> subprocess.CompletedProcess[str]:
    with tempfile.TemporaryDirectory() as directory:
        report = Path(directory) / "result_list.txt"
        report.write_text(content, encoding="utf-8")
        return subprocess.run(
            [sys.executable, str(GATE), "--input", str(report)],
            check=False,
            capture_output=True,
            text=True,
        )


def run_gate(rows: list[str]) -> subprocess.CompletedProcess[str]:
    return run_raw_report(HEADER + "".join(rows))


def assert_gate_accepts_only_zero_defect_export() -> None:
    result = run_gate([result_row("NULL_PTR", "Green", "Unreviewed")])
    assert result.returncode == 0, result.stdout + result.stderr
    assert "unresolved=0" in result.stdout
    assert "justified=0" in result.stdout
    assert "noncompiling=0" in result.stdout


def assert_gate_accepts_crlf_export() -> None:
    content = HEADER + result_row("NULL_PTR", "Green", "Unreviewed")
    result = run_raw_report(content.replace("\n", "\r\n"))
    assert result.returncode == 0, result.stdout + result.stderr


def assert_gate_rejects_justified_defect() -> None:
    result = run_gate(
        [
            result_row(
                "Return of non-const handle to encapsulated data member",
                "Red",
                "Justified",
                function="dt::Dynamics::dtDynSpec::Bodies()",
                file="/builds/dtdynamics/src/dtDynSpec.cpp",
                line=1300,
            )
        ]
    )
    assert result.returncode == 1, result.stdout + result.stderr
    assert "unresolved=1" in result.stdout
    assert "justified=1" in result.stdout


def assert_gate_rejects_green_justified_row() -> None:
    result = run_gate([result_row("NULL_PTR", "Green", "Justified")])
    assert result.returncode == 1, result.stdout + result.stderr
    assert "unresolved=1" in result.stdout
    assert "justified=1" in result.stdout


def assert_gate_rejects_other_reviewed_defects() -> None:
    result = run_gate(
        [
            result_row("MISSING_CONSTEXPR", "Red", "No action planned"),
            result_row("NULL_PTR", "Orange", "Not a defect"),
        ]
    )
    assert result.returncode == 1, result.stdout + result.stderr
    assert "unresolved=2" in result.stdout


def assert_gate_fails_for_noncompiling_row() -> None:
    result = run_gate([result_row("File does not compile", "Green", "Unreviewed")])
    assert result.returncode == 1, result.stdout + result.stderr
    assert "unresolved=1" in result.stdout
    assert "noncompiling=1" in result.stdout


def assert_gate_fails_for_truncated_canonical_schema() -> None:
    truncated_header = "Color\tCheck\tStatus\tFile\tLine\tFunction\n"
    truncated_row = "Green\tNULL_PTR\tUnreviewed\tsrc/example.cpp\t10\tfunction\n"
    result = run_raw_report(truncated_header + truncated_row)
    assert result.returncode == 2, result.stdout + result.stderr
    assert "canonical TSV header" in result.stderr


def assert_gate_fails_for_unexpected_canonical_column() -> None:
    expanded_header = HEADER.rstrip("\n") + "\tUnexpected\n"
    expanded_row = result_row("NULL_PTR", "Green", "Unreviewed").rstrip("\n") + "\textra\n"
    result = run_raw_report(expanded_header + expanded_row)
    assert result.returncode == 2, result.stdout + result.stderr
    assert "canonical TSV header" in result.stderr


def assert_gate_fails_for_reordered_canonical_columns() -> None:
    reordered_header = HEADER.replace("ID\tFamily\t", "Family\tID\t")
    result = run_raw_report(reordered_header)
    assert result.returncode == 2, result.stdout + result.stderr
    assert "canonical TSV header" in result.stderr


def assert_gate_fails_for_truncated_green_row() -> None:
    result = run_raw_report(HEADER + "1\tDefect\tProgramming\tGreen\n")
    assert result.returncode == 2, result.stdout + result.stderr
    assert "missing value" in result.stderr


def assert_gate_fails_for_whitespace_header() -> None:
    whitespace_header = HEADER.replace("Group\tColor\t", "Group\t \tColor\t")
    shifted_color_row = result_row("NULL_PTR", "Red", "Unreviewed").replace(
        "\tRed\tno\t", "\tRed\tGreen\tno\t"
    )
    result = run_raw_report(whitespace_header + shifted_color_row)
    assert result.returncode == 2, result.stdout + result.stderr
    assert "blank TSV headers" in result.stderr


def assert_gate_fails_for_blank_required_identity() -> None:
    result = run_gate([result_row("", "Green", "Unreviewed")])
    assert result.returncode == 2, result.stdout + result.stderr
    assert "blank required value" in result.stderr


def assert_gate_fails_for_whitespace_required_identity() -> None:
    result = run_gate([result_row("  ", "Green", "Unreviewed")])
    assert result.returncode == 2, result.stdout + result.stderr
    assert "blank required value" in result.stderr


def assert_gate_fails_for_non_tsv_line_separator() -> None:
    green_row = result_row("NULL_PTR", "Green", "Unreviewed").rstrip("\n")
    for separator in ("\r", "\u2028", "\u2029"):
        result = run_raw_report(HEADER + green_row + separator)
        assert result.returncode == 2, result.stdout + result.stderr
        assert "disallowed line separator" in result.stderr


def assert_gate_fails_for_non_tsv_control_characters() -> None:
    for control in ("\x00", "\x01", "\x08", "\x0b", "\x1f", "\x7f", "\x80", "\x85", "\x9f"):
        row = result_row("NULL_PTR", "Green", "Unreviewed").replace(
            "Impact: Medium", f"Impact: {control} Medium"
        )
        result = run_raw_report(HEADER + row)
        assert result.returncode == 2, result.stdout + result.stderr
        assert "disallowed control character" in result.stderr


def assert_gate_accepts_literal_quote_field_data() -> None:
    result = run_gate([result_row("NULL_PTR", "Green", "Unreviewed", comment='bare"quote')])
    assert result.returncode == 0, result.stdout + result.stderr
    assert "unresolved=0" in result.stdout


def assert_gate_rejects_quoted_embedded_tab() -> None:
    result = run_gate([result_row("NULL_PTR", "Green", "Unreviewed", comment='"embedded\ttab"')])
    assert result.returncode == 2, result.stdout + result.stderr
    assert "unexpected extra TSV fields" in result.stderr


def assert_gate_rejects_red_row_after_literal_quote() -> None:
    quoted_green = result_row("NULL_PTR", "Green", "Unreviewed", comment='"open')
    red = result_row("NULL_PTR", "Red", "Unreviewed", comment='close"')
    result = run_raw_report(HEADER + quoted_green + red)
    assert result.returncode == 1, result.stdout + result.stderr
    assert "unresolved=1" in result.stdout


def assert_gate_fails_for_missing_declared_trailing_field() -> None:
    truncated_row = result_row("NULL_PTR", "Green", "Unreviewed").removesuffix("\t1\n") + "\n"
    result = run_raw_report(HEADER + truncated_row)
    assert result.returncode == 2, result.stdout + result.stderr
    assert "missing value for: Col" in result.stderr


def assert_gate_fails_for_empty_header() -> None:
    empty_header = HEADER.replace("Group\tColor\t", "Group\t\tColor\t")
    empty_header_row = result_row("NULL_PTR", "Green", "Unreviewed").replace(
        "\tGreen\tno\t", "\t\tGreen\tno\t"
    )
    result = run_raw_report(empty_header + empty_header_row)
    assert result.returncode == 2, result.stdout + result.stderr
    assert "blank TSV headers" in result.stderr


def assert_gate_fails_for_duplicate_header() -> None:
    duplicate_header = HEADER.replace("Color\tNew\t", "Color\tColor\tNew\t")
    duplicate_color_row = result_row("NULL_PTR", "Red", "Unreviewed").replace(
        "\tRed\tno\t", "\tRed\tGreen\tno\t"
    )
    result = run_raw_report(duplicate_header + duplicate_color_row)
    assert result.returncode == 2, result.stdout + result.stderr
    assert "duplicate TSV headers" in result.stderr


def assert_gate_fails_for_extra_tsv_field() -> None:
    valid_row = result_row("NULL_PTR", "Green", "Unreviewed").rstrip("\n")
    result = run_raw_report(HEADER + valid_row + "\textra\n")
    assert result.returncode == 2, result.stdout + result.stderr
    assert "unexpected extra TSV fields" in result.stderr


def assert_gate_fails_for_non_green_rows() -> None:
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
    assert_gate_accepts_only_zero_defect_export()
    assert_gate_accepts_crlf_export()
    assert_gate_rejects_justified_defect()
    assert_gate_rejects_green_justified_row()
    assert_gate_rejects_other_reviewed_defects()
    assert_gate_fails_for_noncompiling_row()
    assert_gate_fails_for_truncated_canonical_schema()
    assert_gate_fails_for_unexpected_canonical_column()
    assert_gate_fails_for_reordered_canonical_columns()
    assert_gate_fails_for_truncated_green_row()
    assert_gate_fails_for_whitespace_header()
    assert_gate_fails_for_blank_required_identity()
    assert_gate_fails_for_whitespace_required_identity()
    assert_gate_fails_for_non_tsv_line_separator()
    assert_gate_fails_for_non_tsv_control_characters()
    assert_gate_accepts_literal_quote_field_data()
    assert_gate_rejects_quoted_embedded_tab()
    assert_gate_rejects_red_row_after_literal_quote()
    assert_gate_fails_for_missing_declared_trailing_field()
    assert_gate_fails_for_empty_header()
    assert_gate_fails_for_duplicate_header()
    assert_gate_fails_for_extra_tsv_field()
    assert_gate_fails_for_non_green_rows()
    print("polyspace result gate tests: PASS")


if __name__ == "__main__":
    main()
