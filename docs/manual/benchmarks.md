# Benchmarks and Resource Reports {#dtdynamics_benchmarks}

The benchmark suite reports both runtime and resource-consumption diagnostics.

## Main binaries

- `benchmark_dtDynamics`: native CCRBA benchmark plus setup/off-loop `dtDynModel_resource` and `dtDynData_resource` rows.
- `benchmark_dtDynamicsStages`: per-stage timing for FK, CCRBA substages, Jacobians, and WBC-style reduced update paths.
- `benchmark_dtDynamicsReferences`: MuJoCo and Pinocchio reference timing/error rows for comparable quantities.

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
build/native-validation/bin/benchmark_dtDynamics > /tmp/native.csv
build/native-validation/bin/benchmark_dtDynamicsStages > /tmp/stages.csv
build/native-validation/bin/benchmark_dtDynamicsReferences > /tmp/references.csv
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


## Current benchmark evidence snapshot

@ref md_docs_benchmark_results contains the longer benchmark report and generated figures. The current manual keeps only a compact evidence snapshot so the API contract is not tied to stale performance numbers.

From the recorded real-XML LeoQuad fixture snapshot, the combined reduced-CCRBA-plus-indexed-Jacobian benchmark reported batch-summary medians of `6.210 us` for nominal, `6.503 us` for asymmetric, and `7.830 us` for nonzero-velocity states. In the recorded installed-package headless LeoQuad profile snapshot, `Robot::UpdateDynamics` mean runtime was `15.589 us`, including `12.663 us` native mean split into `10.232 us` CCRBA and `2.432 us` indexed Jacobians, plus `2.794 us` controller-facing export.

These values are evidence snapshots, not release-wide guarantees. They depend on compiler flags, CPU scheduling, model XML, selected CCRBA term mask, indexed body/site task set, and whether the process runs with CPU isolation or realtime priority. For new comparisons, regenerate CSVs and tables from the benchmark binaries instead of mixing historic and current rows.

The current benchmark resource extension also emits setup rows. `dtDynModel_resource` rows measure `CompileSpec(spec)`, and `dtDynData_resource` rows measure setup-time construction plus `dtDynData::Resize(model)` for each fixture, separately from realtime compute rows. Use them to size initialization cost and memory behavior; do not include them in per-tick controller timing.

Setup inspection rows are reported in microseconds for timing columns. Human-facing tables omit all-zero compute allocation columns; allocation counters remain in CSV output for regression/debug use.


Allocation counters in this benchmark are implemented by overriding C++ `operator new` / `operator new[]` in the benchmark executable, including aligned forms. They are useful for detecting C++ allocation in the measured scope, but they are not a complete replacement for OS-level heap tracing: dtMath dynamic containers can allocate through `malloc`/`calloc`/`realloc`, external libraries may use direct C allocation APIs or allocator pools, allocator-internal behavior is outside this counter, and one-time runtime initialization outside the measured scope is not characterized by these columns. Use a malloc-family interposer or fail-on-allocation guard for final realtime heap certification.

For the current fresh-`dtDynData` resource rows, allocation mean and max are equal because each iteration resizes a fresh data object against the same fixed model and allocates the same set of buffers. Keeping both columns is still useful: the mean summarizes expected setup behavior, while the max would reveal conditional or sporadic allocation regressions. A min allocation column would add little for the current deterministic setup path, but could be added later if resource rows begin mixing reused-data or conditional-allocation cases.
