#!/usr/bin/env python3
"""Classify Polyspace result-list rows for the CI quality gate."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path


RESOLVED_STATUSES = {"Justified", "No action planned", "Not a defect"}
APPROVED_CHECK = "Return of non-const handle to encapsulated data member"
APPROVED_ACCESSOR_LOCATIONS = {
    ("dt::Dynamics::dtDynSpec::Bodies()", "1300"),
    ("dt::Dynamics::dtDynSpec::Joints()", "1305"),
    ("dt::Dynamics::dtDynSpec::Sites()", "1310"),
    ("dt::Dynamics::dtDynSpec::Actuators()", "1315"),
}
REQUIRED_COLUMNS = {"Color", "Check", "Status", "File", "Line", "Function"}


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="Polyspace result_list.txt path")
    return parser.parse_args()


def is_approved_accessor_justification(row: dict[str, str]) -> bool:
    source_file = row.get("File", "").replace("\\", "/")
    return (
        row.get("Check") == APPROVED_CHECK
        and (row.get("Function"), row.get("Line")) in APPROVED_ACCESSOR_LOCATIONS
        and (source_file == "src/dtDynSpec.cpp" or source_file.endswith("/src/dtDynSpec.cpp"))
    )


def classify_results(report: Path) -> tuple[list[dict[str, str]], int, int]:
    with report.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream, delimiter="\t")
        if reader.fieldnames is None:
            raise ValueError("result list is missing a TSV header")
        missing = REQUIRED_COLUMNS.difference(reader.fieldnames)
        if missing:
            raise ValueError(f"result list is missing required columns: {', '.join(sorted(missing))}")

        unresolved: list[dict[str, str]] = []
        justified = 0
        noncompiling = 0
        approved_locations_seen: set[tuple[str | None, str | None]] = set()
        for row in reader:
            if row.get("Check") == "File does not compile":
                noncompiling += 1
                unresolved.append(row)
                continue
            approved_location = (row.get("Function"), row.get("Line"))
            if row.get("Status") == "Justified" and is_approved_accessor_justification(row):
                if approved_location in approved_locations_seen:
                    unresolved.append(row)
                else:
                    approved_locations_seen.add(approved_location)
                    justified += 1
                continue
            if row.get("Status") in RESOLVED_STATUSES:
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
        f"unresolved={len(unresolved)} approved_justified={justified} noncompiling={noncompiling}"
    )
    for row in unresolved:
        print(
            "  "
            f"{row['Check']} | {row['Function']} | {row['File']}:{row['Line']} "
            f"| status={row['Status'] or 'Unreviewed'}"
        )
    return 1 if unresolved else 0


if __name__ == "__main__":
    raise SystemExit(main())
