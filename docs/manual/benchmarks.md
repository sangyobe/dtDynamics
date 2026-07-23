# Benchmarks and Resource Reports {#dtdynamics_benchmarks}

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

The same Part A timing figures include MuJoCo and Pinocchio reference call sequences. Those libraries do not expose an identical CCRBA output contract, so the figures are contextual timing comparisons; status-bearing reference-error rows define which numerical quantities are comparable.

The native CSV `scope` column distinguishes all three native rows. Historical CSVs without `scope` are interpreted as full CCRBA. RNEA reports centroidal-matrix and CoM-velocity consistency as `nan`; reduced CCRBA validates centroidal momentum but reports CoM-velocity consistency as `nan`; full CCRBA validates both.

`fixed_body_heavy` is a one-DOF fixed-topology stress fixture: the canonical 11-body morphology contains a world-fixed base, eight nested fixed mounts, and one terminal revolute link. Native dtDynamics and MuJoCo retain the nested body topology. The hand-built Pinocchio reference folds the fixed offsets into the terminal joint placement and therefore performs less body-recursion work; its timing is representation-specific rather than a same-morphology throughput comparison. The reported `nbody=11` remains the canonical fixture metadata.

### Part B: LeoQuad controller scope

The controller-shaped stage uses

```cpp
DTDYN_CCRBA_TERM_COM_POS |
DTDYN_CCRBA_TERM_CMAMAT |
DTDYN_CCRBA_TERM_CENT_MOM
```

`leoquad_reduced_ccrba` times only that dynamics call. `leoquad_wbc_update` adds indexed Jacobians for the selected root body and four foot sites. External LeoQuad XML timing is opt-in and does not make LeoQuad an installed-package dependency.

## Resource columns

Latency tables report mean, median, and tail timing. The p99 statistic is computed as an empirical order statistic. Let the samples sorted by runtime be \f$t^{\uparrow}_0, \ldots, t^{\uparrow}_{N-1}\f$. The reported tail value is

\f[
k = \mathrm{floor}(0.99(N - 1)), \qquad T_{99} = t^{\uparrow}_k.
\f]

Compute rows contain:

- `allocation_mean`: mean counted C++ `operator new` / `operator new[]` allocation count per measured compute iteration.
- `allocation_max`: maximum counted C++ `operator new` / `operator new[]` allocation count in any measured compute iteration.
- `allocation_bytes_mean`: mean counted C++ `operator new` / `operator new[]` allocated bytes per measured compute iteration.
- `allocation_bytes_max`: maximum counted C++ `operator new` / `operator new[]` allocated bytes in any measured compute iteration.

The setup/off-loop resource sections use similarly named allocation columns. `dtDynModel_resource` rows measure `CompileSpec(spec)`, and `dtDynData_resource` rows measure setup-time construction plus `dtDynData data; data.Resize(model);` iterations, not realtime compute iterations. Units are: `mean_us`, `median_us`, and `p99_us` are microseconds; `allocation_mean` and `allocation_max` are counted allocation-call counts; `allocation_bytes_mean` and `allocation_bytes_max` are counted requested bytes. These columns are benchmark C++ allocation-counter columns, not full malloc-family heap-profiler columns.

Human-facing benchmark tables may omit allocation columns when all rows in the displayed scope are zero. This is presentation-only: the raw CSV sections keep allocation counters so allocation regressions remain machine-checkable.

## Recommended validation command

```bash
cd /path/to/dtDynamics
cmake --preset native-validation
cmake --build --preset native-validation -- -j2
export DTDYNAMICS_BENCHMARK_CPU=0
export DTDYNAMICS_BENCHMARK_BATCHES=7
taskset -c "$DTDYNAMICS_BENCHMARK_CPU" build/native-validation/bin/benchmark_dtDynamics > /tmp/native.csv
taskset -c "$DTDYNAMICS_BENCHMARK_CPU" build/native-validation/bin/benchmark_dtDynamicsStages > /tmp/stages.csv
taskset -c "$DTDYNAMICS_BENCHMARK_CPU" build/native-validation/bin/benchmark_dtDynamicsReferences > /tmp/references.csv
```

Then generate tables/figures from those fresh CSVs:

```bash
python3 tools/plot_benchmark_results.py \
  --native /tmp/native.csv \
  --stages /tmp/stages.csv \
  --references /tmp/references.csv \
  --out /tmp/dtdyn_figures \
  --tables-out /tmp/dtdyn_tables
```

With multi-batch CSVs, the plotting tool uses the median of the per-batch mean, median, and p99 summaries for human tables and figures. It leaves single-batch/historical CSV behavior unchanged. The raw CSV retains the first timing row plus batch-summary min/max spread. Fixing one logical CPU reduces migration noise, but CPU affinity is not frequency locking or realtime isolation; record the governor and scheduling conditions with published numbers.

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

In the fresh canonical real-XML scope, reduced-CCRBA medians were `5.128 us`, `5.373 us`, and `6.884 us` for nominal, asymmetric, and nonzero-velocity states. Reduced CCRBA plus indexed Jacobians reported `7.673 us`, `9.096 us`, and `9.416 us`. These are 200-iteration evidence snapshots. The separately recorded installed-package headless profile remains useful end-to-end evidence, but it is not mixed into the fresh dynamics-scope table.

These values are evidence snapshots, not release-wide guarantees. They depend on compiler flags, CPU scheduling, model XML, selected CCRBA term mask, indexed body/site task set, and whether the process runs with CPU isolation or realtime priority. For new comparisons, regenerate CSVs and tables from the benchmark binaries instead of mixing historic and current rows.

The current benchmark resource extension also emits setup rows. `dtDynModel_resource` rows measure `CompileSpec(spec)`, and `dtDynData_resource` rows measure setup-time construction plus `dtDynData::Resize(model)` for each fixture, separately from realtime compute rows. Use them to size initialization cost and memory behavior; do not include them in per-tick controller timing.

Setup inspection rows are reported in microseconds for timing columns. Human-facing tables omit all-zero compute allocation columns; allocation counters remain in CSV output for regression/debug use.


Allocation counters in this benchmark are implemented by overriding C++ `operator new` / `operator new[]` in the benchmark executable, including aligned forms. They are useful for detecting C++ allocation in the measured scope, but they are not a complete replacement for OS-level heap tracing: dtMath dynamic containers can allocate through `malloc`/`calloc`/`realloc`, external libraries may use direct C allocation APIs or allocator pools, allocator-internal behavior is outside this counter, and one-time runtime initialization outside the measured scope is not characterized by these columns. Use a malloc-family interposer or fail-on-allocation guard for final realtime heap certification.

For the current fresh-`dtDynData` resource rows, allocation mean and max are equal because each iteration resizes a fresh data object against the same fixed model and allocates the same set of buffers. Keeping both columns is still useful: the mean summarizes expected setup behavior, while the max would reveal conditional or sporadic allocation regressions. A min allocation column would add little for the current deterministic setup path, but could be added later if resource rows begin mixing reused-data or conditional-allocation cases.
