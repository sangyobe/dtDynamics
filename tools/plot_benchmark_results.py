#!/usr/bin/env python3
"""Generate dtDynamics benchmark figures from CSV outputs."""

import argparse
import csv
import math
from pathlib import Path

try:
    import matplotlib.pyplot as plt
except ModuleNotFoundError:  # Tables can still be generated on minimal validation hosts.
    plt = None


def read_rows(path):
    rows = []
    section = None
    with open(path, newline="") as stream:
        for raw in csv.reader(stream):
            if not raw:
                continue
            if raw[0] == "fixture":
                section = "native_or_stage"
                header = raw
                continue
            if raw[0] == "engine":
                section = "reference_timing"
                header = raw
                continue
            if raw[0] == "comparison_reference":
                section = "reference_error"
                header = raw
                continue
            if raw[0] == "benchmark_sample":
                if len(raw) > 1 and raw[1] == "executable":
                    continue
                sample_header = ["record_type", "executable", "engine_or_stage", "fixture", "state", "quantity_scope_or_stage", "sample_index", "runtime_us", "alloc_count"]
                item = dict(zip(sample_header, raw))
                item["_section"] = "benchmark_sample"
                rows.append(item)
                continue
            if raw[0] in {"dtDynModel_resource", "dtDynModel_compile", "dtDynData_resource"}:
                if len(raw) > 1 and raw[1] == "fixture":
                    section = None
                    continue
                resource_header = [
                    "record_type", "fixture", "nq", "nv", "nbody", "iterations",
                    "mean_us", "median_us", "p99_us", "allocation_mean", "allocation_max",
                    "allocation_bytes_mean", "allocation_bytes_max",
                ]
                item = dict(zip(resource_header, raw))
                item["_section"] = "dtdynmodel_resource" if raw[0] in {"dtDynModel_resource", "dtDynModel_compile"} else "dtdyndata_resource"
                rows.append(item)
                section = None
                continue
            if raw[0].startswith("benchmark_"):
                section = None
                continue
            if section is None:
                continue
            item = dict(zip(header, raw))
            item["_section"] = section
            rows.append(item)
    return rows


def as_float(row, key):
    try:
        return float(row[key])
    except (KeyError, ValueError):
        return math.nan


def as_int(row, key):
    try:
        return int(row[key])
    except (KeyError, ValueError):
        return 0


def fixture_label(name):
    labels = {
        "single_manipulator_fixed_base": "single arm",
        "dual_manipulator_fixed_base": "dual arm",
        "multi_robot_fixed_base": "multi fixed",
        "multi_robot_floating_base": "multi floating",
        "quadruped_floating_base": "quadruped",
        "fixed_body_heavy": "fixed heavy",
        "serial_chain_10": "chain-10",
        "serial_chain_30": "chain-30",
    }
    return labels.get(name, name.replace("_", " "))




def display_engine_label(name):
    labels = {
        "native": "proposed",
        "pinocchio": "Pinocchio",
        "pinocchio_independent": "Pinocchio independent",
        "mujoco": "MuJoCo",
    }
    return labels.get(name, name.replace("_", " "))


def save_runtime_by_fixture(native_rows, reference_rows, out_dir):
    if plt is None:
        return
    native = {
        row["fixture"]: as_float(row, "median_us")
        for row in native_rows
        if row.get("state") == "nominal"
    }
    references = {}
    for row in reference_rows:
        if row.get("state") != "nominal":
            continue
        references[(row["engine"], row["fixture"])] = as_float(row, "median_us")

    fixtures = list(native.keys())
    engines = ["native", "mujoco", "pinocchio"]
    colors = {"native": "#1f77b4", "mujoco": "#2ca02c", "pinocchio": "#d62728"}
    x = list(range(len(fixtures)))
    width = 0.24

    fig, ax = plt.subplots(figsize=(11.0, 5.2))
    for engine_index, engine in enumerate(engines):
        values = []
        for fixture in fixtures:
            if engine == "native":
                values.append(native.get(fixture, math.nan))
            else:
                values.append(references.get((engine, fixture), math.nan))
        offset = (engine_index - 1) * width
        ax.bar([pos + offset for pos in x], values, width=width, label=display_engine_label(engine), color=colors[engine])

    ax.set_title("Nominal Runtime By Fixture")
    ax.set_ylabel("median runtime (us)")
    ax.set_xticks(x)
    ax.set_xticklabels([fixture_label(name) for name in fixtures], rotation=25, ha="right")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(out_dir / "runtime_by_fixture.png", dpi=180)
    plt.close(fig)


def save_scaling_by_dof(native_rows, reference_rows, out_dir):
    if plt is None:
        return
    series = {"native": [], "mujoco": [], "pinocchio": []}
    for row in native_rows:
        if row.get("state") == "nominal":
            series["native"].append((as_int(row, "nv"), as_float(row, "median_us"), fixture_label(row["fixture"])))
    for row in reference_rows:
        if row.get("state") == "nominal":
            series.setdefault(row["engine"], []).append(
                (as_int(row, "nv"), as_float(row, "median_us"), fixture_label(row["fixture"]))
            )

    fig, ax = plt.subplots(figsize=(8.6, 5.0))
    for engine, values in series.items():
        values = sorted(values)
        if not values:
            continue
        ax.plot([item[0] for item in values], [item[1] for item in values], marker="o", label=display_engine_label(engine))
        for nv, median, label in values:
            ax.annotate(label, (nv, median), textcoords="offset points", xytext=(4, 4), fontsize=7)

    ax.set_title("Runtime Scaling By Velocity Dimension")
    ax.set_xlabel("nv")
    ax.set_ylabel("nominal median runtime (us)")
    ax.grid(alpha=0.25)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(out_dir / "scaling_by_dof.png", dpi=180)
    plt.close(fig)


def save_stage_breakdown(stage_rows, out_dir):
    if plt is None:
        return
    fixtures = ["multi_robot_fixed_base", "multi_robot_floating_base", "quadruped_floating_base"]
    stages = [
        "forward_kinematics",
        "ccrba_prerequisites_after_fk",
        "ccrba_extended_after_prereq",
        "indexed_jacobians_after_ccrba",
        "compute_dynamics_rnea",
        "compute_dynamics_ccrba",
        "leoquad_wbc_update",
    ]
    stage_labels = {
        "forward_kinematics": "FK",
        "ccrba_prerequisites_after_fk": "CCRBA prereq",
        "ccrba_extended_after_prereq": "CCRBA extra",
        "indexed_jacobians_after_ccrba": "indexed J",
        "compute_dynamics_rnea": "RNEA",
        "compute_dynamics_ccrba": "CCRBA",
        "leoquad_wbc_update": "WBC update",
    }
    normal_stage_rows = [row for row in stage_rows if row.get("_section") == "native_or_stage"]
    data = {
        (row["fixture"], row["stage"]): as_float(row, "median_us")
        for row in normal_stage_rows
        if row.get("state") == "nominal"
    }

    x = list(range(len(stages)))
    width = 0.24
    fig, ax = plt.subplots(figsize=(11.0, 5.2))
    for fixture_index, fixture in enumerate(fixtures):
        values = [data.get((fixture, stage), math.nan) for stage in stages]
        offset = (fixture_index - 1) * width
        ax.bar([pos + offset for pos in x], values, width=width, label=fixture_label(fixture))

    ax.set_title("Nominal Stage Breakdown")
    ax.set_ylabel("median runtime (us)")
    ax.set_xticks(x)
    ax.set_xticklabels([stage_labels[stage] for stage in stages], rotation=25, ha="right")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(out_dir / "stage_breakdown.png", dpi=180)
    plt.close(fig)


def save_reference_error(error_rows, out_dir):
    if plt is None:
        return
    quantities = ["Mmat", "cdof_dot", "site_jacp_dot", "site_jacr_dot", "cmAmat", "centMom"]
    fixtures = []
    values = {}
    for row in error_rows:
        if row.get("comparison_reference", row.get("reference", "")) != "mujoco" or row.get("state") != "nonzero_velocity":
            continue
        if row.get("status") != "ok" or row.get("quantity") not in quantities:
            continue
        fixture = row["fixture"]
        if fixture not in fixtures:
            fixtures.append(fixture)
        values[(fixture, row["quantity"])] = as_float(row, "max_error")

    fig, ax = plt.subplots(figsize=(10.5, 5.0))
    x = list(range(len(fixtures)))
    width = 0.12
    for quantity_index, quantity in enumerate(quantities):
        offset = (quantity_index - (len(quantities) - 1) / 2.0) * width
        data = [values.get((fixture, quantity), 0.0) for fixture in fixtures]
        ax.bar([pos + offset for pos in x], data, width=width, label=quantity)

    ax.set_title("MuJoCo Reference Errors For Derivative-Like Outputs")
    ax.set_ylabel("max abs error")
    ax.set_xticks(x)
    ax.set_xticklabels([fixture_label(name) for name in fixtures], rotation=25, ha="right")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False, ncol=3)
    fig.tight_layout()
    fig.savefig(out_dir / "reference_error_summary.png", dpi=180)
    plt.close(fig)


def latex_escape(value):
    return str(value).replace("_", r"\_")


def write_text(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)


def save_runtime_same_scope_table(reference_rows, out_dir):
    scope = "wbc_full_m_c_mdot_centroidal_com"
    fixtures = ["quadruped_floating_base", "multi_robot_floating_base", "single_manipulator_fixed_base"]
    engines = ["native", "pinocchio", "pinocchio_independent", "mujoco"]
    rows = [row for row in reference_rows if row.get("_section") == "reference_timing"]
    lines = [
        r"\begin{tabular}{llrr}",
        r"\toprule",
        r"Fixture/state & Engine & Median us & p99 us \\",
        r"\midrule",
    ]
    any_row = False
    for fixture in fixtures:
        for state in ["nominal", "nonzero_velocity"]:
            for engine in engines:
                candidates = [row for row in rows if row.get("fixture") == fixture and row.get("state") == state and row.get("engine") == engine]
                if engine in {"native", "pinocchio_independent"}:
                    candidates = [row for row in candidates if row.get("quantity_scope") == scope]
                if not candidates:
                    continue
                row = candidates[0]
                lines.append(
                    f"{latex_escape(fixture)}/{latex_escape(state)} & {latex_escape(display_engine_label(engine))} & "
                    f"{as_float(row, 'median_us'):.3f} & {as_float(row, 'p99_us'):.3f} " + "\\\\")
                any_row = True
    if not any_row:
        lines.append(r"\multicolumn{4}{c}{Same-output benchmark rows not generated yet.} \\")
    lines += [r"\bottomrule", r"\end{tabular}", ""]
    write_text(out_dir / "runtime_same_scope.tex", "\n".join(lines))


def save_validation_summary_table(error_rows, out_dir):
    rows = [row for row in error_rows if row.get("_section") == "reference_error"]
    ok_rows = [row for row in rows if row.get("status") == "ok"]
    skipped = [row for row in rows if row.get("status") == "skipped"]
    failed = [row for row in rows if row.get("status") == "fail"]

    groups = [
        ("mujoco", "dynamics", {"Mmat", "qfrc_grav", "qfrc_bias", "qfrc_coriolis", "qfrc_nonlinear"}),
        ("mujoco", "kinematics/J", {"body_xpos", "body_xmat", "site_xpos", "site_jacp", "site_jacr", "site_jacp_dot", "site_jacr_dot"}),
        ("mujoco", "centroidal/CoM", {"subtree_com", "comJmat", "cmAmat", "centMom"}),
        ("pinocchio", "dynamics", {"Mmat", "qfrc_grav", "qfrc_coriolis", "qfrc_nonlinear", "Cmat"}),
        ("pinocchio", "centroidal", {"cmAmat", "centMom"}),
    ]
    lines = [
        r"\begin{tabular}{llrrr}",
        r"\toprule",
        r"Reference & Quantity group & OK & Max abs. & Skipped \\",
        r"\midrule",
    ]
    any_row = False
    for ref, label, quantities in groups:
        ref_ok = [row for row in ok_rows if row.get("comparison_reference", row.get("reference", "")) == ref and row.get("quantity") in quantities]
        ref_skip = [row for row in skipped if row.get("comparison_reference", "") == ref and row.get("quantity") in quantities]
        if not ref_ok and not ref_skip:
            continue
        max_abs = max([as_float(row, "max_error") for row in ref_ok], default=0.0)
        lines.append(f"{latex_escape(ref)} & {label} & {len(ref_ok)} & {max_abs:.1e} & {len(ref_skip)} " + r"\\")
        any_row = True
    if not any_row:
        lines.append(r"\multicolumn{5}{c}{Validation rows not generated yet.} \\")
    if failed:
        lines.append(r"\midrule")
        lines.append(r"\multicolumn{5}{l}{One or more rows exceed tolerance; inspect generated CSV.} \\")
    lines += [r"\bottomrule", r"\end{tabular}", ""]
    write_text(out_dir / "validation_summary.tex", "\n".join(lines))

def save_latency_tail_table(reference_rows, stage_rows, out_dir):
    timing_rows = []
    for row in reference_rows:
        if row.get("_section") == "reference_timing" and row.get("fixture") == "quadruped_floating_base" and row.get("state") == "nonzero_velocity":
            if row.get("engine") in {"native", "pinocchio_independent", "pinocchio", "mujoco"}:
                timing_rows.append((display_engine_label(row.get("engine")), row))
    for row in [item for item in stage_rows if item.get("_section") == "native_or_stage"]:
        if row.get("stage") == "leoquad_wbc_update" and row.get("state") == "nonzero_velocity":
            label = f"reduced WBC/{fixture_label(row.get('fixture', 'unknown'))}"
            timing_rows.append((label, row))
    lines = [
        r"\begin{tabular}{lrr}",
        r"\toprule",
        r"Path & Median us & p99 us \\",
        r"\midrule",
    ]
    if not timing_rows:
        lines.append(r"\multicolumn{3}{c}{Latency rows not generated yet.} \\")
    for label, row in timing_rows:
        lines.append(f"{latex_escape(label)} & {as_float(row, 'median_us'):.3f} & {as_float(row, 'p99_us'):.3f} " + r"\\")
    lines += [r"\bottomrule", r"\end{tabular}", ""]
    write_text(out_dir / "latency_tail_summary.tex", "\n".join(lines))

def save_latency_cdf(reference_rows, stage_rows, out_dir):
    if plt is None:
        return
    sample_rows = [row for row in reference_rows + stage_rows if row.get("_section") == "benchmark_sample"]
    fig, ax = plt.subplots(figsize=(7.2, 4.6))
    plotted = False
    if sample_rows:
        groups = {}
        for row in sample_rows:
            label = row.get("engine_or_stage") or row.get("stage") or row.get("engine") or "unknown"
            label = display_engine_label(label)
            if row.get("fixture") != "quadruped_floating_base" or row.get("state") != "nonzero_velocity":
                continue
            groups.setdefault(label, []).append(as_float(row, "runtime_us"))
        for label, samples in groups.items():
            samples = sorted(v for v in samples if not math.isnan(v))
            if not samples:
                continue
            y = [(i + 1) / len(samples) for i in range(len(samples))]
            ax.plot(samples, y, label=label)
            plotted = True
    if not plotted:
        summary = [row for row in reference_rows if row.get("_section") == "reference_timing" and row.get("fixture") == "quadruped_floating_base" and row.get("state") == "nonzero_velocity"]
        labels = [display_engine_label(row.get("engine", "unknown")) for row in summary]
        med = [as_float(row, "median_us") for row in summary]
        p99 = [as_float(row, "p99_us") for row in summary]
        x = list(range(len(labels)))
        ax.bar([i - 0.18 for i in x], med, width=0.36, label="median")
        ax.bar([i + 0.18 for i in x], p99, width=0.36, label="p99")
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=25, ha="right")
        ax.set_ylabel("runtime (us)")
    else:
        ax.set_ylabel("empirical CDF")
        ax.set_xlabel("runtime (us)")
    ax.set_title("Quadruped Nonzero-Velocity Latency")
    ax.grid(alpha=0.25)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(out_dir / "latency_cdf.png", dpi=180)
    plt.close(fig)



def save_sota_morphology_table(native_rows, reference_rows, out_dir):
    fixtures = [
        "single_manipulator_fixed_base",
        "dual_manipulator_fixed_base",
        "multi_robot_floating_base",
        "quadruped_floating_base",
        "fixed_body_heavy",
        "serial_chain_10",
        "serial_chain_30",
    ]
    native = {row.get("fixture"): row for row in native_rows
              if row.get("_section") == "native_or_stage" and row.get("state") == "nominal"}
    ref = {(row.get("engine"), row.get("fixture")): row for row in reference_rows
           if row.get("_section") == "reference_timing" and row.get("state") == "nominal"}
    lines = [
        r"\begin{tabular}{lrrrr}",
        r"\toprule",
        r"Fixture & $n_v$ & Proposed med. & Pin. med. & MuJoCo med. \\",
        r"\midrule",
    ]
    any_row = False
    for fixture in fixtures:
        row = native.get(fixture)
        if row is None:
            continue
        pin = ref.get(("pinocchio", fixture)) or ref.get(("pinocchio_independent", fixture))
        mj = ref.get(("mujoco", fixture))
        lines.append(
            f"{latex_escape(fixture_label(fixture))} & {row.get('nv', 'n/a')} & "
            f"{as_float(row, 'median_us'):.3f} & "
            f"{as_float(pin, 'median_us'):.3f}" if pin else
            f"{latex_escape(fixture_label(fixture))} & {row.get('nv', 'n/a')} & {as_float(row, 'median_us'):.3f} & --"
        )
        if mj:
            lines[-1] += f" & {as_float(mj, 'median_us'):.3f} " + r"\\"
        else:
            lines[-1] += r" & -- \\"
        any_row = True
    if not any_row:
        lines.append(r"\multicolumn{5}{c}{SOTA morphology rows not generated yet.} \\")
    lines += [r"\bottomrule", r"\end{tabular}", ""]
    write_text(out_dir / "sota_morphology_summary.tex", "\n".join(lines))


def save_sota_ablation_table(stage_rows, out_dir):
    fixture = "quadruped_floating_base"
    state = "nominal"
    stages = [
        ("ccrba_prerequisites_after_fk", "prereq"),
        ("ccrba_centroidal_matrices_after_prereq", "centroidal"),
        ("ccrba_mass_matrix_dot_after_prereq", "$\\dot M$"),
        ("indexed_jacobians_after_ccrba", "indexed $J$"),
        ("compute_dynamics_ccrba_wbc_terms", "WBC mask"),
        ("compute_dynamics_ccrba_all_terms", "all terms"),
    ]
    by_stage = {row.get("stage"): row for row in stage_rows
                if row.get("_section") == "native_or_stage" and row.get("fixture") == fixture and row.get("state") == state}
    lines = [
        r"\begin{tabular}{lrr}",
        r"\toprule",
        r"Stage & Median us & p99 us \\",
        r"\midrule",
    ]
    any_row = False
    for stage, label in stages:
        row = by_stage.get(stage)
        if row is None:
            continue
        lines.append(f"{label} & {as_float(row, 'median_us'):.3f} & {as_float(row, 'p99_us'):.3f} " + r"\\")
        any_row = True
    if not any_row:
        lines.append(r"\multicolumn{3}{c}{SOTA ablation rows not generated yet.} \\")
    lines += [r"\bottomrule", r"\end{tabular}", ""]
    write_text(out_dir / "sota_ablation_summary.tex", "\n".join(lines))


def save_sota_tail_table(reference_rows, stage_rows, out_dir):
    rows = []
    for row in reference_rows:
        if row.get("_section") == "reference_timing" and row.get("fixture") in {"quadruped_floating_base", "multi_robot_floating_base"} and row.get("state") == "nonzero_velocity":
            if row.get("engine") in {"native", "pinocchio", "mujoco"}:
                rows.append((row.get("fixture"), display_engine_label(row.get("engine")), row))
    for row in stage_rows:
        if row.get("_section") == "native_or_stage" and row.get("stage") == "leoquad_wbc_update" and row.get("state") == "nonzero_velocity":
            rows.append((row.get("fixture"), "reduced WBC", row))
    lines = [
        r"\begin{tabular}{llr}",
        r"\toprule",
        r"Fixture & Path & p99 us \\",
        r"\midrule",
    ]
    if not rows:
        lines.append(r"\multicolumn{3}{c}{SOTA tail rows not generated yet.} \\")
    for fixture, label, row in rows[:12]:
        lines.append(f"{latex_escape(fixture_label(fixture))} & {latex_escape(label)} & {as_float(row, 'p99_us'):.3f} " + r"\\")
    lines += [r"\bottomrule", r"\end{tabular}", ""]
    write_text(out_dir / "sota_tail_summary.tex", "\n".join(lines))


def save_setup_resource_table(resource_rows, out_dir):
    compile_rows = {row.get("fixture"): row for row in resource_rows if row.get("_section") == "dtdynmodel_resource"}
    resize_rows = {row.get("fixture"): row for row in resource_rows if row.get("_section") == "dtdyndata_resource"}
    fixtures = sorted(set(compile_rows) | set(resize_rows))
    lines = [
        r"\begin{tabular}{lrrrr}",
        r"\toprule",
        r"Fixture & Compile med. us & Compile p99 us & Resize med. us & Resize p99 us \\",
        r"\midrule",
    ]
    if not fixtures:
        lines.append(r"\multicolumn{5}{c}{setup resource rows not generated yet.} \\")
    for fixture in fixtures:
        compile_row = compile_rows.get(fixture, {})
        resize_row = resize_rows.get(fixture, {})
        lines.append(
            f"{latex_escape(fixture_label(fixture))} & "
            f"{as_float(compile_row, 'median_us'):.3f} & {as_float(compile_row, 'p99_us'):.3f} & "
            f"{as_float(resize_row, 'median_us'):.3f} & {as_float(resize_row, 'p99_us'):.3f} " + r"\\")
    lines += [r"\bottomrule", r"\end{tabular}", ""]
    write_text(out_dir / "setup_resource_summary.tex", "\n".join(lines))

def save_dtdyndata_resource_table(resource_rows, out_dir):
    rows = [row for row in resource_rows if row.get("_section") == "dtdyndata_resource"]
    lines = [
        r"\begin{tabular}{lrrrr}",
        r"\toprule",
        r"Fixture & Resize med. us & Resize p99 us & Resize alloc. mean & Resize bytes mean \\",
        r"\midrule",
    ]
    if not rows:
        lines.append(r"\multicolumn{5}{c}{dtDynData resource rows not generated yet.} \\")
    for row in rows:
        lines.append(
            f"{latex_escape(fixture_label(row.get('fixture', 'unknown')))} & "
            f"{as_float(row, 'median_us'):.3f} & {as_float(row, 'p99_us'):.3f} & "
            f"{as_float(row, 'allocation_mean'):.1f} & {as_float(row, 'allocation_bytes_mean'):.0f} " + r"\\")
    lines += [r"\bottomrule", r"\end{tabular}", ""]
    write_text(out_dir / "dtdyndata_resource_summary.tex", "\n".join(lines))

def save_tables(reference_timing, reference_error, stage_rows, tables_out, native_rows_for_tables, resource_rows):
    tables_out.mkdir(parents=True, exist_ok=True)
    save_runtime_same_scope_table(reference_timing, tables_out)
    save_validation_summary_table(reference_error, tables_out)
    save_latency_tail_table(reference_timing, stage_rows, tables_out)
    save_sota_morphology_table(native_rows_for_tables, reference_timing, tables_out)
    save_sota_ablation_table(stage_rows, tables_out)
    save_sota_tail_table(reference_timing, stage_rows, tables_out)
    save_setup_resource_table(resource_rows, tables_out)
    save_dtdyndata_resource_table(resource_rows, tables_out)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native", required=True, type=Path)
    parser.add_argument("--stages", required=True, type=Path)
    parser.add_argument("--references", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--tables-out", type=Path)
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)

    native_rows = read_rows(args.native)
    stage_rows = read_rows(args.stages)
    reference_rows_all = read_rows(args.references)
    reference_timing = [row for row in reference_rows_all if row["_section"] == "reference_timing"]
    reference_error = [row for row in reference_rows_all if row["_section"] == "reference_error"]
    resource_rows = [row for row in native_rows if row.get("_section") in {"dtdynmodel_resource", "dtdyndata_resource"}]
    native_timing_rows = [row for row in native_rows if row.get("_section") == "native_or_stage"]

    if plt is None:
        print("warning: matplotlib is not installed; skipping figure generation and writing tables only if requested")
    save_runtime_by_fixture(native_timing_rows, reference_timing, args.out)
    save_scaling_by_dof(native_timing_rows, reference_timing, args.out)
    save_stage_breakdown(stage_rows, args.out)
    save_reference_error(reference_error, args.out)
    save_latency_cdf(reference_rows_all, stage_rows, args.out)
    if args.tables_out is not None:
        save_tables(reference_timing, reference_error, stage_rows, args.tables_out, native_timing_rows, resource_rows)


if __name__ == "__main__":
    main()
