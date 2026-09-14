#!/usr/bin/env python3
"""Fail-closed structural validation for dtDynamics benchmark CSV/figure outputs."""

import argparse
import csv
from pathlib import Path


REQUIRED_FIXTURES = {
    "multi_robot_fixed_base",
    "multi_robot_floating_base",
    "fixed_body_heavy",
    "serial_chain_10",
    "serial_chain_30",
}
REQUIRED_STATES = {"nominal", "asymmetric", "nonzero_velocity"}
REQUIRED_SCOPES = {
    "native_rnea_com_pos",
    "native_ccrba_reduced",
    "native_ccrba_all",
}
REQUIRED_STAGES = {
    "forward_kinematics",
    "compute_dynamics_rnea_com_pos",
    "compute_dynamics_ccrba_all_terms",
}


def timing_rows(path: Path, discriminator: str):
    rows = []
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or "median_us" not in reader.fieldnames or "p95_us" not in reader.fieldnames:
            raise ValueError(f"{path}: primary header must contain median_us and p95_us")
        for row in reader:
            if row.get("fixture") and row.get(discriminator):
                rows.append(row)
    return rows


def require_combinations(rows, discriminator, values):
    available = {
        (row["fixture"], row["state"], row[discriminator])
        for row in rows
        if row.get("fixture") in REQUIRED_FIXTURES
    }
    missing = {
        (fixture, state, value)
        for fixture in REQUIRED_FIXTURES
        for state in REQUIRED_STATES
        for value in values
    } - available
    if missing:
        preview = ", ".join(str(item) for item in sorted(missing)[:8])
        raise ValueError(f"missing benchmark combinations: {preview}")


def validate_generated(directory: Path):
    csv_names = ["runtime_by_fixture.csv", "scaling_by_dof.csv", "stage_breakdown.csv"]
    png_names = ["runtime_by_fixture.png", "scaling_by_dof.png", "stage_breakdown.png"]
    for name in csv_names:
        path = directory / name
        if not path.is_file() or path.stat().st_size == 0:
            raise ValueError(f"missing generated CSV: {path}")
        with path.open(newline="") as stream:
            header = next(csv.reader(stream), [])
        if "median_us" not in header or "p95_us" not in header:
            raise ValueError(f"{path}: generated header lacks median_us/p95_us")
    for name in png_names:
        path = directory / name
        if not path.is_file() or path.stat().st_size == 0:
            raise ValueError(f"missing generated figure: {path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native", required=True, type=Path)
    parser.add_argument("--stages", required=True, type=Path)
    parser.add_argument("--generated", required=True, type=Path)
    args = parser.parse_args()

    native = timing_rows(args.native, "scope")
    stages = timing_rows(args.stages, "stage")
    require_combinations(native, "scope", REQUIRED_SCOPES)
    require_combinations(stages, "stage", REQUIRED_STAGES)
    validate_generated(args.generated)
    print("PASS: benchmark result structure and generated artifacts validated")


if __name__ == "__main__":
    main()
