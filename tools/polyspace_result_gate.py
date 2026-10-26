#!/usr/bin/env python3
"""Fail closed unless canonical Polyspace TSV has only Green blank/Unreviewed rows.

An optional, explicit --setup-phase-deviation file narrows the policy for the setup-phase
heap rules only: rows whose check number and file suffix both appear in that file are
counted separately as "deviated" instead of "unresolved". Everything else, including the
same check in any other file, still fails. Without the option the gate is strict.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


CANONICAL_COLUMNS = (
    "ID",
    "Family",
    "Group",
    "Color",
    "New",
    "Check",
    "Information",
    "Class",
    "Function",
    "File",
    "Status",
    "Severity",
    "Comment",
    "Key",
    "Line",
    "Col",
)
REQUIRED_NONEMPTY_COLUMNS = {"Color", "Check", "File", "Line", "Function"}
DISALLOWED_UNICODE_LINE_SEPARATORS = ("\u2028", "\u2029")


def reject_non_tsv_control_characters(content: str) -> None:
    for character in content:
        codepoint = ord(character)
        if character in {"\t", "\n", "\r"}:
            continue
        if codepoint < 0x20 or 0x7F <= codepoint <= 0x9F:
            raise ValueError("result list has a disallowed control character")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="Polyspace result_list.txt path")
    parser.add_argument(
        "--setup-phase-deviation",
        type=Path,
        default=None,
        help="Optional provisional deviation list: '<check-number> <path-suffix>' per line",
    )
    return parser.parse_args()


CHECK_NUMBER = re.compile(r"^(?:Dir )?\d+\.\d+(?:\.\d+)?")


def load_deviation_table(path: Path) -> set[tuple[str, str]]:
    allowed: set[tuple[str, str]] = set()
    with path.open("r", encoding="utf-8") as stream:
        for number, raw_line in enumerate(stream, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 2:
                raise ValueError(f"deviation list line {number} must be '<check> <path-suffix>'")
            check, suffix = parts
            if not re.fullmatch(r"21\.6\.[123]", check):
                raise ValueError(f"deviation list line {number}: only 21.6.1/21.6.2/21.6.3 may be deviated")
            if suffix.startswith("/") or ".." in suffix.split("/"):
                raise ValueError(f"deviation list line {number}: path must be project-relative")
            allowed.add((check, suffix))
    if not allowed:
        raise ValueError("deviation list is empty")
    return allowed


def is_deviated(row: dict[str, str], allowed: set[tuple[str, str]]) -> bool:
    match = CHECK_NUMBER.match(row.get("Check", ""))
    if match is None:
        return False
    check = match.group(0)
    file_path = row.get("File", "")
    return any(check == c and (file_path == s or file_path.endswith("/" + s)) for c, s in allowed)


def split_tsv_physical_lines(content: str) -> list[str]:
    reject_non_tsv_control_characters(content)
    if "\r" in content.replace("\r\n", ""):
        raise ValueError("result list has a disallowed line separator")
    if any(separator in content for separator in DISALLOWED_UNICODE_LINE_SEPARATORS):
        raise ValueError("result list has a disallowed line separator")

    physical_lines = content.replace("\r\n", "\n").split("\n")
    if physical_lines[-1] == "":
        physical_lines.pop()
    return physical_lines


def parse_tsv_record(line: str, line_number: int) -> list[str]:
    del line_number
    return line.split("\t")


def classify_results(
    report: Path, allowed: set[tuple[str, str]] | None = None
) -> tuple[list[dict[str, str]], int, int, list[dict[str, str]]]:
    with report.open("r", encoding="utf-8-sig", newline="") as stream:
        physical_lines = split_tsv_physical_lines(stream.read())
    if not physical_lines:
        raise ValueError("result list is empty")

    headers = parse_tsv_record(physical_lines[0], 1)
    if any(not name.strip() for name in headers):
        raise ValueError("result list has blank TSV headers")
    duplicate_headers = sorted({name for name in headers if headers.count(name) > 1})
    if duplicate_headers:
        raise ValueError(f"result list has duplicate TSV headers: {', '.join(duplicate_headers)}")
    if headers != list(CANONICAL_COLUMNS):
        raise ValueError("result list does not have the canonical TSV header/schema")

    unresolved: list[dict[str, str]] = []
    deviated: list[dict[str, str]] = []
    justified = 0
    noncompiling = 0
    for line_number, line in enumerate(physical_lines[1:], start=2):
        values = parse_tsv_record(line, line_number)
        if len(values) > len(headers):
            raise ValueError(f"result list row {line_number} has unexpected extra TSV fields")
        if len(values) < len(headers):
            missing_values = ", ".join(headers[len(values) :])
            raise ValueError(f"result list row {line_number} is missing value for: {missing_values}")
        row = dict(zip(headers, values))
        blank_values = sorted(column for column in REQUIRED_NONEMPTY_COLUMNS if not row[column].strip())
        if blank_values:
            raise ValueError(
                f"result list row {line_number} has blank required value: {', '.join(blank_values)}"
            )
        if row.get("Check") == "File does not compile":
            noncompiling += 1
            unresolved.append(row)
            continue
        status = row.get("Status")
        if status == "Justified":
            justified += 1
        if status not in {"", "Unreviewed"}:
            unresolved.append(row)
            continue
        if row.get("Color") == "Green":
            continue
        if allowed is not None and is_deviated(row, allowed):
            deviated.append(row)
            continue
        unresolved.append(row)
    return unresolved, justified, noncompiling, deviated


def main() -> int:
    arguments = parse_arguments()
    try:
        allowed = (
            load_deviation_table(arguments.setup_phase_deviation)
            if arguments.setup_phase_deviation is not None
            else None
        )
        unresolved, justified, noncompiling, deviated = classify_results(arguments.input, allowed)
    except (OSError, ValueError) as error:
        print(f"ERROR: cannot evaluate Polyspace result list: {error}", file=sys.stderr)
        return 2

    print(
        "Polyspace result gate: "
        f"unresolved={len(unresolved)} justified={justified} noncompiling={noncompiling}"
    )
    if allowed is not None:
        by_file: dict[str, int] = {}
        for row in deviated:
            by_file[row["File"].rsplit("/", 1)[-1]] = by_file.get(row["File"].rsplit("/", 1)[-1], 0) + 1
        print(f"Polyspace result gate: deviated={len(deviated)} (provisional setup-phase) {by_file}")
    for row in unresolved:
        print(
            "  "
            f"{row['Check']} | {row['Function']} | {row['File']}:{row['Line']} "
            f"| color={row['Color'] or 'Unset'} status={row['Status'] or 'Unreviewed'}"
        )
    return 1 if unresolved else 0


if __name__ == "__main__":
    raise SystemExit(main())
