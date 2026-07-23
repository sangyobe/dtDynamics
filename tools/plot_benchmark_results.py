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
            if raw[0] == "benchmark_batch_summary":
                if len(raw) > 1 and raw[1] == "benchmark":
                    batch_header = raw
                    section = None
                    continue
                item = dict(zip(batch_header, raw))
                item["_section"] = "batch_summary"
                rows.append(item)
                section = None
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


def prefer_batch_summaries(rows, benchmark_name):
    """Promote robust batch medians into timing rows when summaries exist.

    The raw first-batch rows and batch min/max fields remain in the source CSV.
    Single-batch and historical CSVs are returned unchanged.
    """
    summaries = {
        (row["fixture"], row["state"], row["stage"]): row
        for row in rows
        if row.get("_section") == "batch_summary" and row.get("benchmark") == benchmark_name
    }
    if not summaries:
        return rows

    promoted = []
    for row in rows:
        key = None
        if benchmark_name == "benchmark_dtDynamics" and row.get("_section") == "native_or_stage":
            key = (row["fixture"], row["state"], row.get("scope", "native_ccrba_all"))
        elif benchmark_name == "benchmark_dtDynamicsStages" and row.get("_section") == "native_or_stage":
            key = (row["fixture"], row["state"], row["stage"])
        elif benchmark_name == "benchmark_dtDynamicsReferences" and row.get("_section") == "reference_timing":
            key = (row["fixture"], row["state"], f'{row["engine"]}:{row["quantity_scope"]}')

        summary = summaries.get(key)
        if summary is None:
            promoted.append(row)
            continue

        item = dict(row)
        item["mean_us"] = summary["mean_us_median"]
        item["median_us"] = summary["median_us_median"]
        item["p99_us"] = summary["p99_us_median"]
        item["_batch_aggregated"] = "true"
        item["_batches"] = summary["batches"]
        item["_median_us_min"] = summary["median_us_min"]
        item["_median_us_max"] = summary["median_us_max"]
        item["_p99_us_min"] = summary["p99_us_min"]
        item["_p99_us_max"] = summary["p99_us_max"]
        promoted.append(item)
    return promoted


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
        "leoquad_qd1": "LeoQuad QD1",
        "fixed_body_heavy": "fixed_body_heavy",
        "serial_chain_10": "chain-10",
        "serial_chain_30": "chain-30",
    }
    return labels.get(name, name.replace("_", " "))


def native_ccrba_rows(rows):
    """Return all-feature CCRBA rows for native-vs-reference figures.

    CSVs from before the explicit scope column are treated as CCRBA-all for
    backward compatibility.
    """
    return [
        row for row in rows
        if row.get("scope", "native_ccrba_all") == "native_ccrba_all"
    ]



def display_engine_label(name, native_display_label="dtDynamics"):
    labels = {
        "native": native_display_label,
        "native_rnea_com_pos": f"{native_display_label} RNEA + CoM",
        "native_ccrba_reduced": f"{native_display_label} reduced CCRBA",
        "native_ccrba_all": f"{native_display_label} full CCRBA",
        "pinocchio": "Pinocchio",

        "mujoco": "MuJoCo",
    }
    return labels.get(name, name.replace("_", " "))


PART_A_COLORS = {
    "native_rnea_com_pos": "#1f77b4",
    "native_ccrba_reduced": "#17becf",
    "native_ccrba_all": "#9467bd",
    "mujoco": "#2ca02c",
    "pinocchio": "#d62728",
}


def save_runtime_by_fixture(native_rows, reference_rows, out_dir, native_display_label="dtDynamics"):
    if plt is None:
        return
    native = {
        (row.get("scope", "native_ccrba_all"), row["fixture"]): as_float(row, "median_us")
        for row in native_rows
        if row.get("state") == "nominal"
    }
    references = {}
    for row in reference_rows:
        if row.get("state") != "nominal":
            continue
        references[(row["engine"], row["fixture"])] = as_float(row, "median_us")

    fixtures = []
    for row in native_rows:
        if row.get("state") == "nominal" and row["fixture"] not in fixtures:
            fixtures.append(row["fixture"])
    engines = [
        "native_rnea_com_pos",
        "native_ccrba_reduced",
        "native_ccrba_all",
        "mujoco",
        "pinocchio",
    ]

    x = list(range(len(fixtures)))
    width = 0.16

    fig, ax = plt.subplots(figsize=(12.4, 5.4))
    for engine_index, engine in enumerate(engines):
        values = []
        for fixture in fixtures:
            if engine.startswith("native_"):
                values.append(native.get((engine, fixture), math.nan))
            else:
                values.append(references.get((engine, fixture), math.nan))
        offset = (engine_index - (len(engines) - 1) / 2.0) * width
        ax.bar([pos + offset for pos in x], values, width=width,
               label=display_engine_label(engine, native_display_label), color=PART_A_COLORS[engine])

    ax.set_title("Part A Nominal Dynamics Runtime By Fixture")
    ax.set_ylabel("median runtime (us)")
    ax.set_xticks(x)
    ax.set_xticklabels([fixture_label(name) for name in fixtures], rotation=25, ha="right")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False, ncol=2)
    fig.tight_layout()
    fig.savefig(out_dir / "runtime_by_fixture.png", dpi=180)
    plt.close(fig)

def save_scaling_by_dof(native_rows, reference_rows, out_dir, native_display_label="dtDynamics"):
    if plt is None:
        return
    series = {
        "native_rnea_com_pos": [],
        "native_ccrba_reduced": [],
        "native_ccrba_all": [],
        "mujoco": [],
        "pinocchio": [],
    }
    for row in native_rows:
        if row.get("state") == "nominal":
            scope = row.get("scope", "native_ccrba_all")
            if scope in series:
                series[scope].append((as_int(row, "nv"), as_float(row, "median_us"), fixture_label(row["fixture"])))
    for row in reference_rows:
        if row.get("state") == "nominal" and row.get("engine") in {"mujoco", "pinocchio"}:
            series[row["engine"]].append(
                (as_int(row, "nv"), as_float(row, "median_us"), fixture_label(row["fixture"]))
            )

    fig, ax = plt.subplots(figsize=(9.6, 5.2))
    for engine, values in series.items():
        values = sorted(values)
        if not values:
            continue
        ax.plot([item[0] for item in values],
                [item[1] for item in values],
                marker="o",
                label=display_engine_label(engine, native_display_label),
                color=PART_A_COLORS[engine])

    ax.set_title("Part A Runtime Scaling By Velocity Dimension")
    ax.set_xlabel("nv")
    ax.set_ylabel("nominal median runtime (us)")
    ax.grid(alpha=0.25)
    ax.legend(frameon=False, ncol=2)
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
        "compute_dynamics_rnea_com_pos",
        "compute_dynamics_ccrba_all_terms",
        "leoquad_reduced_ccrba",
        "leoquad_wbc_update",
    ]
    stage_labels = {
        "forward_kinematics": "FK",
        "ccrba_prerequisites_after_fk": "CCRBA prereq",
        "ccrba_extended_after_prereq": "CCRBA extra",
        "indexed_jacobians_after_ccrba": "indexed J",
        "compute_dynamics_rnea_com_pos": "RNEA + CoM",
        "compute_dynamics_ccrba_all_terms": "CCRBA all",
        "leoquad_reduced_ccrba": "reduced CCRBA",
        "leoquad_wbc_update": "reduced + indexed J",
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


def latex_escape(value):
    return str(value).replace("_", r"\_")


def write_text(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)


def save_runtime_same_scope_table(reference_rows, out_dir, native_display_label="dtDynamics"):
    scope = "wbc_full_m_c_mdot_centroidal_com"
    fixtures = ["quadruped_floating_base", "multi_robot_floating_base", "single_manipulator_fixed_base"]
    engines = ["native", "pinocchio", "mujoco"]
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
                if engine == "native":
                    candidates = [row for row in candidates if row.get("quantity_scope") == scope]
                if not candidates:
                    continue
                row = candidates[0]
                lines.append(
                    f"{latex_escape(fixture)}/{latex_escape(state)} & "
                    f"{latex_escape(display_engine_label(engine, native_display_label))} & "
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

def save_latency_tail_table(reference_rows, stage_rows, out_dir, native_display_label="dtDynamics"):
    timing_rows = []
    for row in reference_rows:
        if row.get("_section") == "reference_timing" and row.get("fixture") == "quadruped_floating_base" and row.get("state") == "nonzero_velocity":
            if row.get("engine") in {"native", "pinocchio", "mujoco"}:
                timing_rows.append((display_engine_label(row.get("engine"), native_display_label), row))
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

def save_latency_cdf(reference_rows, stage_rows, out_dir, native_display_label="dtDynamics"):
    if plt is None:
        return
    sample_rows = [row for row in reference_rows + stage_rows if row.get("_section") == "benchmark_sample"]
    fig, ax = plt.subplots(figsize=(7.2, 4.6))
    plotted = False
    if sample_rows:
        groups = {}
        for row in sample_rows:
            engine = row.get("engine_or_stage") or row.get("engine") or "unknown"
            if engine not in {"native", "pinocchio", "mujoco"}:
                continue
            label = display_engine_label(engine, native_display_label)
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
        summary = [row for row in reference_rows
                   if row.get("_section") == "reference_timing"
                   and row.get("fixture") == "quadruped_floating_base"
                   and row.get("state") == "nonzero_velocity"
                   and row.get("engine") in {"native", "pinocchio", "mujoco"}]
        labels = [display_engine_label(row.get("engine", "unknown"), native_display_label) for row in summary]
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
    ax.set_title("Quadruped Nonzero-Velocity Latency Summary")
    ax.grid(alpha=0.25)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(out_dir / "latency_cdf.png", dpi=180)
    plt.close(fig)



def save_sota_morphology_table(native_rows, reference_rows, out_dir, native_display_label="dtDynamics"):
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
        f"Fixture & $n_v$ & {latex_escape(native_display_label)} med. & Pin. med. & MuJoCo med. " + r"\\",
        r"\midrule",
    ]
    any_row = False
    for fixture in fixtures:
        row = native.get(fixture)
        if row is None:
            continue
        pin = ref.get(("pinocchio", fixture))
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
        ("leoquad_reduced_ccrba", "LeoQuad mask"),
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


def save_sota_tail_table(reference_rows, stage_rows, out_dir, native_display_label="dtDynamics"):
    rows = []
    for row in reference_rows:
        if row.get("_section") == "reference_timing" and row.get("fixture") in {"quadruped_floating_base", "multi_robot_floating_base"} and row.get("state") == "nonzero_velocity":
            if row.get("engine") in {"native", "pinocchio", "mujoco"}:
                rows.append((row.get("fixture"), display_engine_label(row.get("engine"), native_display_label), row))
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
        r"\begin{tabular}{lrrrrr}",
        r"\toprule",
        r"Fixture & Compile med. us & Compile p99 us & Resize med. us & Resize p99 us & Resize alloc. \\",
        r"\midrule",
    ]
    if not fixtures:
        lines.append(r"\multicolumn{6}{c}{setup resource rows not generated yet.} \\")
    for fixture in fixtures:
        compile_row = compile_rows.get(fixture, {})
        resize_row = resize_rows.get(fixture, {})
        lines.append(
            f"{latex_escape(fixture_label(fixture))} & "
            f"{as_float(compile_row, 'median_us'):.3f} & {as_float(compile_row, 'p99_us'):.3f} & "
            f"{as_float(resize_row, 'median_us'):.3f} & {as_float(resize_row, 'p99_us'):.3f} & "
            f"{as_float(resize_row, 'allocation_mean'):.0f} " + r"\\")
    lines += [r"\bottomrule", r"\end{tabular}", ""]
    write_text(out_dir / "setup_resource_summary.tex", "\n".join(lines))


def save_tables(reference_timing, reference_error, stage_rows, tables_out,
                native_rows_for_tables, resource_rows, native_display_label="dtDynamics"):
    tables_out.mkdir(parents=True, exist_ok=True)
    obsolete_resource_table = tables_out / "dtdyndata_resource_summary.tex"
    if obsolete_resource_table.exists():
        obsolete_resource_table.unlink()
    save_runtime_same_scope_table(reference_timing, tables_out, native_display_label)
    save_validation_summary_table(reference_error, tables_out)
    save_latency_tail_table(reference_timing, stage_rows, tables_out, native_display_label)
    save_sota_morphology_table(native_rows_for_tables, reference_timing, tables_out, native_display_label)
    save_sota_ablation_table(stage_rows, tables_out)
    save_sota_tail_table(reference_timing, stage_rows, tables_out, native_display_label)
    save_setup_resource_table(resource_rows, tables_out)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native", required=True, type=Path)
    parser.add_argument("--stages", required=True, type=Path)
    parser.add_argument("--references", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--tables-out", type=Path)
    parser.add_argument("--native-display-label", default="dtDynamics")
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)

    native_rows = prefer_batch_summaries(read_rows(args.native), "benchmark_dtDynamics")
    stage_rows = prefer_batch_summaries(read_rows(args.stages), "benchmark_dtDynamicsStages")
    reference_rows_all = prefer_batch_summaries(read_rows(args.references), "benchmark_dtDynamicsReferences")
    reference_timing = [row for row in reference_rows_all if row["_section"] == "reference_timing"]
    reference_error = [row for row in reference_rows_all if row["_section"] == "reference_error"]
    resource_rows = [row for row in native_rows if row.get("_section") in {"dtdynmodel_resource", "dtdyndata_resource"}]
    native_timing_rows = [row for row in native_rows if row.get("_section") == "native_or_stage"]
    native_ccrba_timing_rows = native_ccrba_rows(native_timing_rows)

    if plt is None:
        print("warning: matplotlib is not installed; skipping figure generation and writing tables only if requested")
    save_runtime_by_fixture(native_timing_rows, reference_timing, args.out, args.native_display_label)
    save_scaling_by_dof(native_timing_rows, reference_timing, args.out, args.native_display_label)
    save_stage_breakdown(stage_rows, args.out)
    save_latency_cdf(reference_rows_all, stage_rows, args.out, args.native_display_label)
    if args.tables_out is not None:
        save_tables(reference_timing, reference_error, stage_rows, args.tables_out,
                    native_ccrba_timing_rows, resource_rows, args.native_display_label)


if __name__ == "__main__":
    main()
