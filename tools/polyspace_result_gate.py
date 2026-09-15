#!/usr/bin/env python3
"""Fail closed unless canonical Polyspace TSV has only Green blank/Unreviewed rows."""

from __future__ import annotations

import argparse
import csv
import io
import sys
from pathlib import Path


REQUIRED_COLUMNS = {"Color", "Check", "Status", "File", "Line", "Function"}
REQUIRED_NONEMPTY_COLUMNS = {"Color", "Check", "File", "Line", "Function"}
DISALLOWED_LINE_SEPARATORS = (
    "\x0b",
    "\x0c",
    "\x1c",
    "\x1d",
    "\x1e",
    "\x1f",
    "\x85",
    "\u2028",
    "\u2029",
)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="Polyspace result_list.txt path")
    return parser.parse_args()


def split_tsv_physical_lines(content: str) -> list[str]:
    if "\r" in content.replace("\r\n", ""):
        raise ValueError("result list has a disallowed line separator")
    if any(separator in content for separator in DISALLOWED_LINE_SEPARATORS):
        raise ValueError("result list has a disallowed line separator")

    physical_lines = content.replace("\r\n", "\n").split("\n")
    if physical_lines[-1] == "":
        physical_lines.pop()
    return physical_lines


def parse_tsv_record(line: str, line_number: int) -> list[str]:
    try:
        records = list(csv.reader([line], delimiter="\t", strict=True))
    except csv.Error as error:
        raise ValueError(f"result list row {line_number} has invalid TSV quoting: {error}") from error
    if len(records) != 1:
        raise ValueError(f"result list row {line_number} cannot be parsed as one TSV record")

    values = records[0]
    canonical = io.StringIO()
    csv.writer(canonical, delimiter="\t", lineterminator="", quoting=csv.QUOTE_MINIMAL).writerow(values)
    if canonical.getvalue() != line:
        raise ValueError(f"result list row {line_number} has invalid TSV quoting")
    return values


def classify_results(report: Path) -> tuple[list[dict[str, str]], int, int]:
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
    missing = REQUIRED_COLUMNS.difference(headers)
    if missing:
        raise ValueError(f"result list is missing required columns: {', '.join(sorted(missing))}")

    unresolved: list[dict[str, str]] = []
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
        unresolved.append(row)
    return unresolved, justified, noncompiling


def main() -> int:
    arguments = parse_arguments()
    try:
        unresolved, justified, noncompiling = classify_results(arguments.input)
    except (OSError, ValueError, csv.Error) as error:
        print(f"ERROR: cannot evaluate Polyspace result list: {error}", file=sys.stderr)
        return 2

    print(
        "Polyspace result gate: "
        f"unresolved={len(unresolved)} justified={justified} noncompiling={noncompiling}"
    )
    for row in unresolved:
        print(
            "  "
            f"{row['Check']} | {row['Function']} | {row['File']}:{row['Line']} "
            f"| color={row['Color'] or 'Unset'} status={row['Status'] or 'Unreviewed'}"
        )
    return 1 if unresolved else 0


if __name__ == "__main__":
    raise SystemExit(main())
