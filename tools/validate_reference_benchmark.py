#!/usr/bin/env python3
"""Fail-closed semantic validation for dtDynamics external-reference benchmark CSV."""

import argparse
import csv
import math
from collections import Counter
from pathlib import Path

FIXTURES = {
    "single_manipulator_fixed_base",
    "dual_manipulator_fixed_base",
    "multi_robot_fixed_base",
    "multi_robot_floating_base",
    "quadruped_floating_base",
    "fixed_body_heavy",
    "serial_chain_10",
    "serial_chain_30",
}
STATES = {"nominal", "asymmetric", "nonzero_velocity"}
SCOPES = {
    "native": "wbc_full_m_c_mdot_centroidal_com",
    "mujoco": "forward_fullM_bias_site_jacobians_centroidal",
    "pinocchio": "crba_gravity_nle_coriolis_ccrba_centroidal_dot",
}
MUJOCO_QUANTITIES = {
    "Mmat", "gravityTorq", "coriolisTorq", "nonlinTorq", "cdof", "cdof_dot", "cvel",
    "site_jacp", "site_jacr", "site_jacp_dot", "site_jacr_dot", "subtree_com", "comJmat",
    "cmAmat", "centMom",
}
PIN_CORE = {"Mmat", "gravityTorq", "coriolisTorq", "nonlinTorq"}
PIN_CENTROIDAL = {"cmAmat", "centMom", "cmAmatDot_qvel"}


def require(condition, message):
    if not condition:
        raise SystemExit(f"FAIL: {message}")


def parse(path):
    timing = []
    errors = []
    summaries = []
    metadata = {}
    header = None
    section = None
    with path.open(newline="") as stream:
        for raw in csv.reader(stream):
            if not raw:
                continue
            if raw[0] == "engine":
                header, section = raw, "timing"
                continue
            if raw[0] == "comparison_reference":
                header, section = raw, "errors"
                continue
            if raw[0] == "benchmark_batch_summary" and len(raw) > 1 and raw[1] == "benchmark":
                header, section = raw, "summaries"
                continue
            if raw[0] == "benchmark_metadata":
                require(len(raw) >= 3, "malformed benchmark_metadata row")
                metadata[raw[1]] = raw[2]
                continue
            if raw[0] == "benchmark_sample":
                continue
            if section == "timing" and header and len(raw) == len(header):
                timing.append(dict(zip(header, raw)))
            elif section == "errors" and header and len(raw) == len(header):
                errors.append(dict(zip(header, raw)))
            elif section == "summaries" and header and len(raw) == len(header):
                summaries.append(dict(zip(header, raw)))
    return timing, errors, summaries, metadata


def read_runtime_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--references", required=True, type=Path)
    parser.add_argument("--runtime-csv", required=True, type=Path,
                        help="normalized runtime_by_fixture.csv generated from this reference raw CSV")
    args = parser.parse_args()
    timing, errors, summaries, metadata = parse(args.references)

    require(metadata.get("build_type") == "Release", "reference benchmark must be Release")
    require(metadata.get("batch_count") == "7", "reference benchmark must use seven batches")
    require(metadata.get("warmup_iterations") == "20", "expected 20 warmup iterations")
    require(metadata.get("measured_iterations") == "200", "expected 200 measured iterations")
    require(metadata.get("mujoco_native_validation_runs_during_timing") == "0",
            "missing/nonzero MuJoCo timing independence oracle")
    require(metadata.get("pinocchio_native_validation_runs_during_timing") == "0",
            "missing/nonzero Pinocchio timing independence oracle")
    require(metadata.get("nonfinite_error_policy_self_test") == "pass",
            "missing non-finite comparison-policy self-test")

    expected_keys = {(engine, fixture, state) for engine in SCOPES for fixture in FIXTURES for state in STATES}
    actual_keys = {(row.get("engine"), row.get("fixture"), row.get("state")) for row in timing}
    require(len(timing) == 72 and actual_keys == expected_keys, "timing coverage must be 3 engines x 8 fixtures x 3 states")
    for row in timing:
        require(row.get("quantity_scope") == SCOPES[row["engine"]], f"wrong scope for {row['engine']}")
        require(row.get("iterations") == "200", "timing row iteration mismatch")
        median = float(row["median_us"])
        p95 = float(row["p95_us"])
        p99 = float(row["p99_us"])
        require(0.0 <= median <= p95 <= p99, "invalid median/p95/p99 ordering")

    expected_summary_keys = {
        (engine, fixture, state, f"{engine}:{scope}")
        for engine, scope in SCOPES.items() for fixture in FIXTURES for state in STATES
    }
    summary_keys = {
        (row.get("stage", "").split(":", 1)[0], row.get("fixture"), row.get("state"), row.get("stage"))
        for row in summaries
    }
    require(len(summaries) == 72 and summary_keys == expected_summary_keys,
            "batch-summary coverage must exactly match all 72 timing keys")
    for row in summaries:
        require(row.get("batches") == "7", "batch-summary row must aggregate seven batches")
        for metric in ("mean_us", "median_us", "p95_us", "p99_us"):
            values = [float(row[f"{metric}_{suffix}"]) for suffix in ("min", "median", "max")]
            require(all(math.isfinite(value) for value in values), f"non-finite {metric} batch summary")
            require(0.0 <= values[0] <= values[1] <= values[2], f"invalid {metric} min/median/max ordering")
        require(float(row["median_us_median"]) <= float(row["p95_us_median"]) <= float(row["p99_us_median"]),
                "invalid promoted median/p95/p99 ordering")

    require(len(errors) == 528, "expected 528 explicit numerical/skip rows")
    statuses = Counter(row.get("status") for row in errors)
    require(statuses == Counter({"ok": 465, "skipped": 63}), f"unexpected status counts: {dict(statuses)}")
    require(not any(row.get("status") == "fail" for row in errors), "reference comparison failure present")
    numeric_error_fields = ("max_error", "max_relative_error", "rms_error", "abs_tolerance", "rel_tolerance")
    for row in errors:
        require(all(math.isfinite(float(row[field])) for field in numeric_error_fields),
                "non-finite comparison error or tolerance field")
        require(float(row["abs_tolerance"]) >= 0.0 and float(row["rel_tolerance"]) >= 0.0,
                "negative comparison tolerance")

    error_map = {(row["comparison_reference"], row["fixture"], row["state"], row["quantity"]): row for row in errors}
    expected_error_keys = set()
    for fixture in FIXTURES:
        for state in STATES:
            expected_error_keys.update(("mujoco", fixture, state, quantity) for quantity in MUJOCO_QUANTITIES)
            expected_error_keys.update(("pinocchio", fixture, state, quantity) for quantity in PIN_CORE | PIN_CENTROIDAL)
    require(set(error_map) == expected_error_keys, "comparison quantity coverage mismatch or duplicate rows")

    for key, row in error_map.items():
        reference, fixture, _state, quantity = key
        expected_skip = reference == "pinocchio" and quantity in PIN_CENTROIDAL and fixture != "quadruped_floating_base"
        if expected_skip:
            require(row["status"] == "skipped" and row.get("diagnostic", "").strip(),
                    f"non-equivalent quantity not explicitly skipped: {key}")
            require(row.get("count") == "0", f"skipped row unexpectedly reports compared samples: {key}")
        else:
            require(row["status"] == "ok", f"comparable quantity not validated: {key}")
            require(int(row["count"]) > 0, f"passing row has no compared samples: {key}")
            require(not row.get("diagnostic", "").strip(), f"passing row has unexpected diagnostic: {key}")

    normalized = read_runtime_csv(args.runtime_csv)
    normalized_refs = [row for row in normalized if row.get("engine") in {"mujoco", "pinocchio"}]
    expected_external_keys = {
        (engine, fixture, state)
        for engine in ("mujoco", "pinocchio") for fixture in FIXTURES for state in STATES
    }
    normalized_keys = [
        (row.get("engine"), row.get("fixture"), row.get("state")) for row in normalized_refs
    ]
    require(len(normalized_refs) == 48, "normalized runtime CSV must contain 48 external-reference rows")
    require(len(set(normalized_keys)) == 48, "normalized external-reference keys must be unique")
    require(set(normalized_keys) == expected_external_keys,
            "normalized external-reference coverage must exactly match engine x fixture x state")
    summary_map = {
        (row["stage"].split(":", 1)[0], row["fixture"], row["state"]): row for row in summaries
    }
    for row in normalized_refs:
        key = (row["engine"], row["fixture"], row["state"])
        require(key in summary_map, f"normalized runtime row has no batch summary: {key}")
        summary = summary_map[key]
        for output_field, summary_field in (
            ("mean_us", "mean_us_median"),
            ("median_us", "median_us_median"),
            ("p95_us", "p95_us_median"),
            ("p99_us", "p99_us_median"),
        ):
            require(math.isclose(float(row[output_field]), float(summary[summary_field]), rel_tol=0.0, abs_tol=1.0e-12),
                    f"normalized runtime value not promoted from accepted batch summary: {key} {output_field}")

    print("PASS: independent MuJoCo/Pinocchio benchmark semantics validated")


if __name__ == "__main__":
    main()
