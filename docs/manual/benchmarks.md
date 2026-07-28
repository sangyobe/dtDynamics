# Benchmarks and Resource Reports {#dtdynamics_benchmarks}

Author: Joonhee Jo

이 chapter는 runtime 숫자보다 먼저 **측정 scope와 해석 경계**를 정의한다. native dynamics, independently prepared stage, MuJoCo/Pinocchio reference는 서로 다른 호출 집합이므로 median을 더해 다른 row를 만들거나 same-output speed ratio로 읽지 않는다. 각 batch는 fresh `dtDynData`, deterministic state, prerequisite preparation 뒤 warmup과 measurement를 수행한다.

채택 artifact는 raw CSV, seven-batch summary, normalized CSV, PNG를 함께 검증한다. reference output은 exact engine × fixture × state key coverage, unique key, finite value, `median <= p95 <= p99`, raw summary와 promoted value agreement를 통과해야 한다. wall-clock rerun의 byte identity는 요구하지 않지만 candidate figure/CSV는 자신이 기록한 accepted raw run과 일치해야 한다.

The benchmark suite reports both runtime and resource-consumption diagnostics.

## Main binaries

- `benchmark_dtDynamics`: three dynamics-only native scopes (`native_rnea_com_pos`, `native_ccrba_reduced`, and `native_ccrba_all`) plus setup/off-loop `dtDynModel_resource` and `dtDynData_resource` rows.
- `benchmark_dtDynamicsStages`: per-stage timing for FK, CCRBA substages, Jacobians, and WBC-style reduced update paths.
- `benchmark_dtDynamicsReferences`: MuJoCo and Pinocchio reference timing/error rows for comparable quantities.

## Benchmark scopes

### Part A: native dynamics and external references

Part A contains three package-native dynamics calls for every fixture/state:

- `native_rnea_com_pos` calls `ComputeDynamicsRNEA(model, data, gravity, DTDYN_RNEA_TERM_COM_POS)`.
- `native_ccrba_reduced` calls `ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_COM_POS | DTDYN_CCRBA_TERM_CMAMAT | DTDYN_CCRBA_TERM_CENT_MOM)`.
- `native_ccrba_all` calls `ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_ALL)`.

The same Part A timing figures include MuJoCo and Pinocchio reference call sequences. Those libraries do not expose an identical CCRBA output contract, so the figures are contextual library-overhead timings, not same-output speed ratios; status-bearing reference-error rows define which numerical quantities are comparable. Release-active metadata counters require zero native validation runs during each external timing scope. Native comparison runs execute only after the external timing section.

The native CSV `scope` column distinguishes all three native rows. Historical CSVs without `scope` are interpreted as full CCRBA. RNEA reports centroidal-matrix and CoM-velocity consistency as `nan`; reduced CCRBA validates centroidal momentum but reports CoM-velocity consistency as `nan`; full CCRBA validates both.

The deterministic state labels separate configuration and velocity coverage. `nominal` uses model-default positions with identity free/ball orientations and zero velocity. `asymmetric` applies nonuniform position/orientation perturbations while keeping velocity zero. `nonzero_velocity` uses a different perturbed pose plus nonzero generalized velocities, activating velocity-dependent dynamics; it is not the asymmetric pose with velocity simply added. Reference adapters apply those semantics through their engine-specific coordinate mappings.

`fixed_body_heavy` is a one-DOF fixed-topology stress fixture: the canonical 11-body morphology contains a world-fixed base, eight nested fixed mounts, and one terminal revolute link. Native dtDynamics and MuJoCo retain the nested body topology. The hand-built Pinocchio reference folds the fixed offsets into the terminal joint placement and therefore performs less body-recursion work; its timing is representation-specific rather than a same-morphology throughput comparison. The reported `nbody=11` remains the canonical fixture metadata.

The expanded morphology set also includes `multi_robot_fixed_base`, a branching two-root fixed-base model with articulated branches, a fixed sensor frame, multiple sites, and actuators; `multi_robot_floating_base`, the corresponding two-root free-joint topology; and `serial_chain_10` / `serial_chain_30`, matched serial scaling points. These fixtures separate branching, floating-root, welded-body, and increasing-DOF effects instead of treating `nv` as the only work descriptor.

Every repeated native scope and stage batch constructs and resizes a fresh `dtDynData`, restores the deterministic state, and prepares that stage's prerequisites before warmup and measurement. Stage rows therefore begin from independent prepared state and must not be added to reconstruct another row.

### Part B: LeoQuad controller scope

The controller-shaped stage uses

```cpp
DTDYN_CCRBA_TERM_COM_POS |
DTDYN_CCRBA_TERM_CMAMAT |
DTDYN_CCRBA_TERM_CENT_MOM
```

`leoquad_reduced_ccrba` times only that dynamics call. `leoquad_wbc_update` adds indexed Jacobians for the selected root body and four foot sites. External LeoQuad XML timing is opt-in and does not make LeoQuad an installed-package dependency.

## Resource columns

Latency tables report mean, median, p95, and p99 timing. Both tail statistics are empirical order statistics. Let the samples sorted by runtime be \f$t^{\uparrow}_0, \ldots, t^{\uparrow}_{N-1}\f$. The reported values are

\f[
k_{95} = \mathrm{floor}(0.95(N - 1)), \qquad T_{95} = t^{\uparrow}_{k_{95}},
\qquad
k_{99} = \mathrm{floor}(0.99(N - 1)), \qquad T_{99} = t^{\uparrow}_{k_{99}}.
\f]

Compute rows contain:

- `allocation_mean`: mean counted C++ `operator new` / `operator new[]` allocation count per measured compute iteration.
- `allocation_max`: maximum counted C++ `operator new` / `operator new[]` allocation count in any measured compute iteration.
- `allocation_bytes_mean`: mean counted C++ `operator new` / `operator new[]` allocated bytes per measured compute iteration.
- `allocation_bytes_max`: maximum counted C++ `operator new` / `operator new[]` allocated bytes in any measured compute iteration.

The setup/off-loop resource sections use similarly named allocation columns. `dtDynModel_resource` rows measure `CompileSpec(spec)`, and `dtDynData_resource` rows measure setup-time construction plus `dtDynData data; data.Resize(model);` iterations, not realtime compute iterations. Units are: `mean_us`, `median_us`, `p95_us`, and `p99_us` are microseconds; `allocation_mean` and `allocation_max` are counted allocation-call counts; `allocation_bytes_mean` and `allocation_bytes_max` are counted requested bytes. These columns are benchmark C++ allocation-counter columns, not full malloc-family heap-profiler columns.

Human-facing benchmark tables may omit allocation columns when all rows in the displayed scope are zero. This is presentation-only: the raw CSV sections keep allocation counters so allocation regressions remain machine-checkable.

## Recommended validation command

```bash
cd /path/to/dtDynamics
cmake --preset validation
cmake --build --preset validation -- -j2
export DTDYNAMICS_BENCHMARK_CPU=0
export DTDYNAMICS_BENCHMARK_BATCHES=7
taskset -c "$DTDYNAMICS_BENCHMARK_CPU" build/validation/bin/benchmark_dtDynamics > /tmp/native.csv
taskset -c "$DTDYNAMICS_BENCHMARK_CPU" build/validation/bin/benchmark_dtDynamicsStages > /tmp/stages.csv
taskset -c "$DTDYNAMICS_BENCHMARK_CPU" build/validation/bin/benchmark_dtDynamicsReferences > /tmp/references.csv
```

Then generate tables/figures from those fresh CSVs:

```bash
python3 tools/plot_benchmark_results.py \
  --native /tmp/native.csv \
  --stages /tmp/stages.csv \
  --references /tmp/references.csv \
  --out /tmp/dtdyn_figures \
  --tables-out /tmp/dtdyn_tables

python3 tools/validate_benchmark_results.py \
  --native /tmp/native.csv \
  --stages /tmp/stages.csv \
  --generated /tmp/dtdyn_figures

python3 tools/validate_reference_benchmark.py \
  --references /tmp/references.csv \
  --runtime-csv /tmp/dtdyn_figures/runtime_by_fixture.csv
```

The plot command writes the three human figures and their machine-readable source families side by side: `runtime_by_fixture.{csv,png}`, `scaling_by_dof.{csv,png}`, and `stage_breakdown.{csv,png}`. The CSVs retain mean, median, p95, and p99 values; the figures use medians and leave tail interpretation to the CSV and tables.

With multi-batch CSVs, the plotting tool uses the median of the per-batch mean, median, p95, and p99 summaries for human tables and figures. It leaves single-batch/historical CSV behavior unchanged. The raw CSV retains the first timing row plus batch-summary min/max spread. Fixing one logical CPU reduces migration noise, but CPU affinity is not frequency locking or realtime isolation; record the governor and scheduling conditions with published numbers.

## Publishing generated Doxygen in GitLab

Yes. Generated Doxygen can be published immediately as a GitLab CI job artifact without committing HTML. A private GitLab Pages site is also the preferred stable URL when the self-managed instance has Pages enabled. This project requests private Pages access, but no Pages deployment currently exists and the API response does not prove that the instance-level Pages service is configured; confirm that service with the GitLab administrator before relying on a Pages URL.

Keep generated HTML out of Git. The CI runner must provide Doxygen, Graphviz, and the LaTeX/Ghostscript tools required by this repository's local formula-image configuration. A guaranteed artifact job shape is:

```yaml
doxygen-artifact:
  stage: build
  script:
    - tools/validate_docs.sh
  artifacts:
    paths:
      - build/docs/doxygen/html/
```

When instance-level Pages is confirmed, add `deploy` to the pipeline's `stages` list and publish the generated directory:

```yaml
doxygen-pages:
  stage: deploy
  script:
    - tools/validate_docs.sh
    - mkdir -p public
    - cp -a build/docs/doxygen/html/. public/
  pages:
    publish: public
  rules:
    - if: '$CI_COMMIT_BRANCH == $CI_DEFAULT_BRANCH'
```

Pin a project-approved image or runner toolchain before enabling either job. Validate the first pipeline, download the artifact, and verify private Pages access before treating either output as a release artifact.


## Current benchmark evidence snapshot

@ref md_docs_benchmark_results contains the longer benchmark report and generated figures. The current manual keeps only a compact evidence snapshot so the API contract is not tied to stale performance numbers.

The current canonical evidence is a Release build pinned to CPU 0 with seven batches. Each batch uses 20 warmup iterations and 200 measured iterations. Human-facing values are the medians of the seven batch-level mean, median, p95, and p99 summaries; they are not pooled-sample percentiles.

### Native floating-base quadruped snapshot

The compact Part A snapshot below reproduces the detailed report's floating-base quadruped rows. Each cell is `median / p95` in microseconds.

| State | RNEA + CoM | Reduced CCRBA | Full CCRBA | MuJoCo | Pinocchio |
|---|---:|---:|---:|---:|---:|
| nominal | 5.751 / 5.881 | 5.073 / 5.213 | 6.398 / 6.560 | 6.320 / 6.384 | 17.962 / 18.090 |
| asymmetric | 5.877 / 6.032 | 5.182 / 5.310 | 6.518 / 6.685 | 7.000 / 7.072 | 18.367 / 18.511 |
| nonzero_velocity | 6.049 / 6.187 | 6.907 / 7.045 | 8.166 / 8.293 | 7.039 / 7.119 | 18.364 / 18.507 |

For the same quadruped fixture, the setup-only resource row reported compile median / p99 of `5.263 / 11.240 us`, data-resize median / p99 of `4.249 / 4.983 us`, and 31 counted C++ allocation calls per fresh resize iteration. Setup remains outside the realtime timing loop. The measured native dynamics and stage compute rows reported zero counted C++ `operator new` allocations; this does not cover direct malloc-family paths.

### LeoQuad real-XML snapshot

In the current CPU-affined seven-batch canonical real-XML scope, reduced-CCRBA central medians were `5.162 us`, `5.255 us`, and `6.917 us` for nominal, asymmetric, and nonzero-velocity states. Reduced CCRBA plus indexed Jacobians reported `7.501 us`, `7.630 us`, and `9.213 us`. The separately recorded installed-package headless profile remains useful end-to-end evidence, but it is not mixed into the fresh dynamics-scope table.

These values are evidence snapshots, not release-wide guarantees. They depend on compiler flags, CPU scheduling, model XML, selected CCRBA term mask, indexed body/site task set, CPU affinity, frequency policy, and realtime priority. For new comparisons, regenerate CSVs and tables from the benchmark binaries instead of mixing historic and current rows.

The current benchmark resource extension also emits setup rows. `dtDynModel_resource` rows measure `CompileSpec(spec)`, and `dtDynData_resource` rows measure setup-time construction plus `dtDynData::Resize(model)` for each fixture, separately from realtime compute rows. Use them to size initialization cost and memory behavior; do not include them in per-tick controller timing.

Setup inspection rows are reported in microseconds for timing columns. Human-facing tables omit all-zero compute allocation columns; allocation counters remain in CSV output for regression/debug use.


Allocation counters in this benchmark are implemented by overriding C++ `operator new` / `operator new[]` in the benchmark executable, including aligned forms. They are useful for detecting C++ allocation in the measured scope, but they are not a complete replacement for OS-level heap tracing: dtMath dynamic containers can allocate through `malloc`/`calloc`/`realloc`, external libraries may use direct C allocation APIs or allocator pools, allocator-internal behavior is outside this counter, and one-time runtime initialization outside the measured scope is not characterized by these columns. Use a malloc-family interposer or fail-on-allocation guard for final realtime heap certification.

For the current fresh-`dtDynData` resource rows, allocation mean and max are equal because each iteration resizes a fresh data object against the same fixed model and allocates the same set of buffers. Keeping both columns is still useful: the mean summarizes expected setup behavior, while the max would reveal conditional or sporadic allocation regressions. A min allocation column would add little for the current deterministic setup path, but could be added later if resource rows begin mixing reused-data or conditional-allocation cases.
