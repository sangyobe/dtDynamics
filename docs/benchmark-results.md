# dtDynamics Benchmark Results

Date: 2026-07-16

Updated: 2026-07-16 for the rc3 benchmark/resource schema, refreshed validation figures, and the latest package and headless LeoQuad gates.

This note records the current native `dtDynamics` benchmark baseline and the reference-path timing pass. The reference benchmarks use the private validation cache in the standalone package checkout:

- Package root: this standalone `dtDynamics` checkout
- MuJoCo 3.8.0: `.validation_deps/mujoco-3.8.0`
- Pinocchio 3.9.0: `.validation_deps/pinocchio-3.9.0`

The timing and error scopes are intentionally explicit. Native `dtDynamics` now exposes explicit dynamics entry points: `ComputeDynamicsRNEA` for MuJoCo-step1-like base ingredients and `ComputeDynamicsCCRBA` for the base ingredients plus `Cmat` and optional centroidal outputs. There is no public `ComputeDynamics` alias. RNEA accepts `dtDynRneaTermFlags`; CCRBA accepts `dtDynCcrbaTermFlags` for heavier optional centroidal outputs. Jacobian timing is reported separately in `benchmark_dtDynamicsStages`. MuJoCo and Pinocchio rows time their external reference calls for overlapping ingredients only; they are useful reference-path baselines, not a strict apples-to-apples algorithm comparison yet. The reference benchmark also emits a second CSV table with `comparison_reference,fixture,state,quantity,count,max_error,rms_error,status,diagnostic`.

The derivative-style benchmark figures are scoped to quantities already implemented by native `dtDynamics`: `MmatDot`, `Cmat`, `Jdot`, `Jdot*qvel`, `cmAmatDot`, `comJmatDot`, and related spatial derivatives. This follows the runtime/error-reporting style of analytical dynamics-derivative work such as [arXiv:2302.06001](https://arxiv.org/pdf/2302.06001) and the benchmark categories exposed by [ROAM-Lab-ND/spatial_v2_extended](https://github.com/ROAM-Lab-ND/spatial_v2_extended), but it does not claim second-order dynamics derivative support yet.

## Command

```bash
# Run from the standalone dtDynamics package root.
tools/audit_release_readiness.sh

cmake --preset validation
cmake --build --preset validation -- -j2
ctest --preset validation
build/validation/bin/benchmark_dtDynamics
build/validation/bin/benchmark_dtDynamicsStages
build/validation/bin/benchmark_dtDynamicsReferences
python3 tools/plot_benchmark_results.py \
  --native /tmp/codex_native_multi_floating.csv \
  --stages /tmp/codex_stages_multi_floating.csv \
  --references /tmp/codex_ref_multi_floating_pin.csv \
  --out docs/figures

DTDYNAMICS_BENCHMARK_ONLY_EXTERNAL=1 \
DTDYNAMICS_BENCHMARK_XML=/path/to/LeoQuad_QD1.xml \
DTDYNAMICS_BENCHMARK_XML_NAME=leoquad_qd1 \
  build/validation/bin/benchmark_dtDynamicsStages
```

Validation status for this run: `tools/audit_release_readiness.sh` passed, including `test_dtDynTypes`, `test_dtDynamicsMujocoRef`, `test_dtDynamicsPinocchioRef`, install/export validation, and release-invariant scans.

Important: benchmark numbers must come from a Release build. A default single-config CMake build leaves `CMAKE_BUILD_TYPE` empty, compiling native `dtDynamics` with no `-O` flag while MuJoCo and Pinocchio are optimized prebuilt libraries. That unoptimized configuration made native `dtDynamics` look roughly an order of magnitude slower than it is.

The Pinocchio/Boost CMake warnings observed during validation configure are not production package warnings. They come from the private Pinocchio validation prefix and CMake's Boost discovery policy/runtime-path diagnostics. `benchmark_dtDynamicsReferences` builds and runs to completion, emits both timing and error-summary CSV sections, and resolves MuJoCo/Pinocchio libraries from the standalone package's `.validation_deps` cache. These warnings should not block benchmark use unless the reference executable fails to start or reports nonzero reference errors.

All benchmark executables append a final `benchmark_metadata,key,value` CSV section. It records the executable name, build type, compiler, C++ standard, platform, CPU model, hardware concurrency, clock source, clock steadiness, warmup iterations, measured iterations, batch count, and `NDEBUG` status. This metadata is appended after timing/error rows so existing timing parsers can continue to consume the primary CSV sections by header.

Set `DTDYNAMICS_BENCHMARK_BATCHES=N` for repeated-run aggregation. When `N > 1`, the benchmarks append `benchmark_batch_summary` rows containing min/median/max of per-batch mean, median, and p99 runtime plus the maximum allocation count across batches. The default is one batch, which preserves the compact timing output.

Each row uses 20 warmup iterations and 200 measured iterations. Allocation counts are measured by intercepting global C++ `operator new` / `operator new[]` while the measured call is active; they do not intercept dtMath or library allocations that route directly through `malloc`/`calloc`/`realloc`.

The raw benchmark CSVs retain allocation counters for regression checking. Human-readable timing tables omit allocation columns when every displayed row in that scope is zero; zero allocation is stated once in prose instead of repeated in every row.

## Native Baseline

Native scope: dynamics-only `ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_ALL)`. LeoQuad runtime now requests a smaller mask: `COM_POS | CMAMAT | CENT_MOM`, then computes indexed body/site Jacobians through the indexed `ComputeJacobians` wrapper. The indexed wrapper is intentionally retained instead of switching to `ComputeSiteJacobians`, because LeoQuad still needs the root/base body Jacobian in the same control update.

| Fixture | State | nq | nv | bodies | mean us | median us | p99 us |
|---|---|---:|---:|---:|---:|---:|---:|
| single_manipulator_fixed_base | nominal | 3 | 3 | 5 | 1.568560 | 1.567000 | 1.726000 |
| single_manipulator_fixed_base | asymmetric | 3 | 3 | 5 | 1.331600 | 1.356000 | 1.757000 |
| single_manipulator_fixed_base | nonzero_velocity | 3 | 3 | 5 | 1.326440 | 1.324000 | 1.387000 |
| dual_manipulator_fixed_base | nominal | 4 | 4 | 6 | 1.227340 | 1.226000 | 1.294000 |
| dual_manipulator_fixed_base | asymmetric | 4 | 4 | 6 | 1.341885 | 1.267000 | 1.883000 |
| dual_manipulator_fixed_base | nonzero_velocity | 4 | 4 | 6 | 1.711540 | 1.648000 | 2.696000 |
| multi_robot_fixed_base | nominal | 4 | 4 | 8 | 1.422080 | 1.419000 | 1.487000 |
| multi_robot_fixed_base | asymmetric | 4 | 4 | 8 | 1.472530 | 1.444000 | 1.676000 |
| multi_robot_fixed_base | nonzero_velocity | 4 | 4 | 8 | 1.857435 | 1.857000 | 1.898000 |
| multi_robot_floating_base | nominal | 18 | 16 | 8 | 3.170610 | 3.148000 | 3.498000 |
| multi_robot_floating_base | asymmetric | 18 | 16 | 8 | 3.186170 | 3.169000 | 3.435000 |
| multi_robot_floating_base | nonzero_velocity | 18 | 16 | 8 | 4.195815 | 4.142000 | 5.854000 |
| quadruped_floating_base | nominal | 19 | 18 | 14 | 5.319970 | 5.273000 | 7.671000 |
| quadruped_floating_base | asymmetric | 19 | 18 | 14 | 5.345090 | 5.305000 | 7.539000 |
| quadruped_floating_base | nonzero_velocity | 19 | 18 | 14 | 6.775280 | 6.728000 | 9.476000 |
| fixed_body_heavy | nominal | 1 | 1 | 11 | 1.311965 | 1.312000 | 1.362000 |
| fixed_body_heavy | asymmetric | 1 | 1 | 11 | 1.328080 | 1.311000 | 1.822000 |
| fixed_body_heavy | nonzero_velocity | 1 | 1 | 11 | 1.571690 | 1.569000 | 1.615000 |

For all native rows, these internal consistency errors were reported as zero at benchmark output precision:

- mass-matrix symmetry max/RMS
- `nonlinTorq = coriolisTorq + gravityTorq`
- `centMom = cmAmat * qvel`
- `comVel = comJmat * qvel`

## Figures

The figures are generated from the same CSV outputs as the tables:

\image html runtime_by_fixture.png "Nominal runtime by fixture" width=100%

\image html scaling_by_dof.png "Runtime scaling by velocity dimension" width=100%

\image html stage_breakdown.png "Nominal stage breakdown" width=100%

\image html reference_error_summary.png "MuJoCo reference errors for derivative-like outputs" width=100%

## Native Stage Breakdown

`benchmark_dtDynamicsStages` splits the native pipeline into public API stages and internal CCRBA/RNEA substages. Each stage starts from the prerequisite state needed by that API, so the rows are diagnostic timings rather than additive exclusive profiler samples.

The setup-only `benchmark_dtDynamics` resource rows (`dtDynModel_resource` and `dtDynData_resource`) inspect model compilation and data resize cost outside the realtime loop. These rows are included here to make setup cost explicit; they are not part of the controller update timing. The human-facing table omits allocation columns because compute allocation columns are all zero and setup allocation counts are diagnostic counter values rather than time. The CSV still keeps allocation counters for regression checks.

| Fixture | nq | nv | bodies | Compile median us | Compile p99 us | Data resize median us | Data resize p99 us |
|---|---:|---:|---:|---:|---:|---:|---:|
| single_manipulator_fixed_base | 3 | 3 | 5 | 1.832 | 2.453 | 1.887 | 3.510 |
| dual_manipulator_fixed_base | 4 | 4 | 6 | 2.258 | 2.750 | 2.206 | 3.185 |
| multi_robot_fixed_base | 4 | 4 | 8 | 2.773 | 3.065 | 2.442 | 4.642 |
| multi_robot_floating_base | 18 | 16 | 8 | 3.293 | 3.870 | 3.702 | 5.882 |
| quadruped_floating_base | 19 | 18 | 14 | 6.679 | 9.244 | 5.360 | 7.468 |
| fixed_body_heavy | 1 | 1 | 11 | 2.698 | 3.147 | 2.736 | 4.198 |
| serial_chain_10 | 10 | 10 | 12 | 3.650 | 4.704 | 3.919 | 6.438 |
| serial_chain_30 | 30 | 30 | 32 | 12.455 | 15.673 | 10.312 | 13.829 |

For the floating-base quadruped fixture, the median stage times in microseconds were:

| State | FK | Mass after FK | CCRBA after FK | CCRBA prereq | CCRBA extended | Body Jacobians | Site Jacobians | All Jacobians | Indexed Jacobians | Gravity | RNEA dynamics | CCRBA dynamics |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| nominal | 1.625 | 1.322 | 3.694 | 0.923 | 2.934 | 2.296 | 0.746 | 3.217 | 0.912 | 0.055 | 5.058 | 4.134 |
| asymmetric | 1.740 | 1.294 | 3.763 | 0.896 | 2.889 | 2.253 | 0.741 | 3.226 | 0.896 | 0.055 | 5.193 | 4.461 |
| nonzero_velocity | 1.742 | 1.388 | 6.503 | 2.249 | 2.858 | 2.256 | 0.742 | 3.201 | 0.891 | 0.055 | 5.263 | 5.735 |

A focused run for the multi-root fixtures validates the timing path for independent model parts plus fixed sensor frames. Median stage times in microseconds were:

| Fixture | State | FK | Mass after FK | CCRBA after FK | CCRBA prereq | CCRBA extended | Indexed Jacobians | RNEA dynamics | CCRBA dynamics | WBC Update |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| multi_robot_fixed_base | nominal | 0.689 | 0.461 | 0.741 | 0.425 | 0.298 | 0.100 | 1.850 | 1.192 | 1.364 |
| multi_robot_fixed_base | asymmetric | 0.718 | 0.458 | 0.736 | 0.424 | 0.292 | 0.101 | 1.869 | 1.212 | 1.386 |
| multi_robot_fixed_base | nonzero_velocity | 0.723 | 0.464 | 1.172 | 0.802 | 0.295 | 0.101 | 1.895 | 1.615 | 1.791 |
| multi_robot_floating_base | nominal | 0.737 | 0.858 | 2.664 | 0.571 | 2.138 | 0.643 | 2.653 | 2.292 | 3.245 |
| multi_robot_floating_base | asymmetric | 0.859 | 0.925 | 2.710 | 0.570 | 2.239 | 0.668 | 2.692 | 2.312 | 3.336 |
| multi_robot_floating_base | nonzero_velocity | 0.850 | 0.944 | 3.581 | 1.369 | 2.188 | 0.640 | 2.804 | 3.199 | 4.221 |

A focused rerun after adding the LeoQuad-shaped combined stage reported the following evidence snapshot. Treat these numbers as run-specific timing evidence, not a timeless performance guarantee:

| State | Indexed Jacobians | WBC CCRBA Terms | LeoQuad WBC Update |
| --- | ---: | ---: | ---: |
| nominal | 1.042 | 11.423 | 12.657 |
| asymmetric | 1.033 | 10.683 | 11.776 |
| nonzero_velocity | 1.078 | 10.561 | 11.733 |

`LeoQuad WBC Update` is `ComputeDynamicsCCRBA(model, data, gravity, COM_POS | CMAMAT | CENT_MOM)` followed by indexed body/site `ComputeJacobians(model, data, bodyIds, siteIds)`. In the benchmark fixture, `bodyIds` contains the root body and `siteIds` contains all four foot sites.

The stage benchmark also accepts an opt-in external XML fixture through `DTDYNAMICS_BENCHMARK_XML`. This is used for real LeoQuad timing without making the standalone dtDynamics package depend on LeoQuad assets. `DTDYNAMICS_BENCHMARK_ONLY_EXTERNAL=1` restricts the run to that external fixture, and `DTDYNAMICS_BENCHMARK_XML_NAME` controls the CSV fixture label.

After replacing temporary 6x6 cross-matrix construction in the CCRBA bias-force assembly with direct sparse cross-term accumulation, removing the redundant local `Bmat` scratch array, computing local bias directly into `BmatComp`, fusing the public CCRBA `Cmat*qvel` product into the `Cmat` fill loop, skipping zero-velocity local bias construction, skipping zero bias-force products, avoiding redundant nonzero-body inertia clears, grouping local motion-cross updates by spatial-inertia column, combining paired DOF 6x6-vector products, filling body spatial inertia directly in place, replacing scaled spatial temporaries with in-place scaled adds, caching compiled body/inertial/site rotation matrices, using a data-internal combined indexed `J`/`Jdot` writer for controller-selected body/site Jacobians, moving diff-motion zeroing to exact write sites with exact-zero cross-product skips, writing `cdof` and nonzero `cdof_dot` directly, keeping the local momentum multiply inside `ComputeBiasMatrix` in dtMath operator form after benchmark-confirming no meaningful regression, carrying a per-body `BmatCompNonzero` flag through composite bias accumulation, skipping redundant `BmatComp` clears when the flag prevents reads, writing zero-derivative DOF bias products directly into `f2` with `SpatialMulInto`, and unrolling fixed-size 6x6 spatial matrix-vector products in the DOF bias-force path, representative real LeoQuad XML batch-summary medians are:

| State | CCRBA prereq | Local bias matrices | Bias accumulation | DOF bias forces | CCRBA bias forces | WBC CCRBA Terms | LeoQuad WBC Update |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| nominal | 0.940 | 0.063 | 0.027 | 0.194 | 0.251 | 4.235 | 6.210 |
| asymmetric | 0.991 | 0.064 | 0.027 | 0.194 | 0.251 | 4.404 | 6.503 |
| nonzero_velocity | 2.248 | 0.798 | 0.114 | 0.546 | 1.531 | 5.831 | 7.830 |

The current optimization pass stops at this point to preserve readability. The remaining nonzero local-bias matrix row is the next possible target, but reducing it further would likely require replacing the readable `ComputeBiasMatrix` assembly helpers with a larger hand-expanded formula.

A profiled headless LeoQuad trot validation run with `SIMROBOT_PROFILE_DYNAMICS=1` reported `Robot::UpdateDynamics` timing over the actual control path. After removing redundant exact-size zeroing in controller-facing copy helpers, the current validation run is:

| samples | mean us | min us | max us | last us |
|---:|---:|---:|---:|---:|
| 20103 | 15.589 | 11.131 | 163.781 | 12.415 |

The same run emitted the opt-in split timing:

| input mean us | native mean us | export mean us | input last us | native last us | export last us |
|---:|---:|---:|---:|---:|---:|
| 0.132 | 12.663 | 2.794 | 0.141 | 10.190 | 2.084 |

The native block was further split as:

| CCRBA mean us | Jacobian mean us | CCRBA last us | Jacobian last us |
|---:|---:|---:|---:|
| 10.232 | 2.432 | 8.219 | 1.971 |

`input` covers qpos/qvel staging, local `qVel` copy, and gravity setup. `native` covers the reduced `ComputeDynamicsCCRBA` call plus indexed body/site `ComputeJacobians`. `export` covers LeoQuad-side copying of computed `dtDynData` fields into existing controller-facing members and derived base/task quantities. The split still shows the main remaining runtime cost is reduced CCRBA under the real LeoQuad model/state, but the direct bias-force assembly, local-scratch cleanup, compiled rotation caches, combined indexed Jacobian writer, diff-motion zero fast path, composite bias nonzero flag, and no-temporary DOF bias product keep the latest isolated-prefix headless CCRBA mean near 10.2 us with no measured allocation in the native benchmark rows.

The CCRBA extended stage is further split as:

| State | Dense clear | Root CoM | CMM/CMMdot | Mmat | MmatDot | Cmat | CoM J/Jdot | CMM*qvel | CoM J*qvel | Cmat*qvel | Products |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| nominal | 0.105 | 0.023 | 0.148 | 0.347 | 0.598 | 0.717 | 0.170 | 0.207 | 0.131 | 0.533 | 0.817 |
| asymmetric | 0.115 | 0.024 | 0.150 | 0.344 | 0.618 | 0.689 | 0.169 | 0.203 | 0.126 | 0.501 | 0.775 |
| nonzero_velocity | 0.105 | 0.023 | 0.146 | 0.346 | 0.597 | 0.689 | 0.169 | 0.202 | 0.126 | 0.500 | 0.776 |

The `Cmat*qvel` row remains in the stage benchmark as a diagnostic standalone product. The public `ComputeDynamicsCCRBA` path no longer pays that separate dense pass: it accumulates `coriolisTorq` while filling `Cmat`, then forms `nonlinTorq = gravityTorq + coriolisTorq`.

The old top-level spatial-array pre-clear was removed after confirming downstream stages overwrite the spatial work arrays (`cdof`, `cdof_dot`, `cvel`, `cinert`, `crb`, `BmatComp`, and `f1/f2/f3`). Dense output matrix clearing remains inside the CCRBA path because non-ancestor entries in `Mmat`, `MmatDot`, and `Cmat` are intentionally not written by the parent-chain loops and must remain zero.

The single-manipulator nominal median path was 1.567 us for full `ComputeDynamicsCCRBA(..., DTDYN_CCRBA_TERM_ALL)`; the dual-manipulator nominal path was 1.226 us; the multi-robot fixed-base nominal path was 1.419 us; the multi-robot floating-base nominal path was 3.148 us; the fixed-body-heavy nominal path was 1.312 us. All measured stage compute rows reported zero counted C++ `operator new` allocation; the raw stage CSV retains allocation counters for regression checks.

This explains most of the remaining native-vs-MuJoCo timing difference. `DTDYN_CCRBA_TERM_ALL` computes controller-specific quantities beyond the MuJoCo overlap set. LeoQuad does not need all of those terms each control tick, so its runtime call requests only `COM_POS`, `CMAMAT`, and `CENT_MOM`; the stage benchmark includes this as `compute_dynamics_ccrba_wbc_terms`. The stage benchmark also includes `leoquad_wbc_update`, which runs the reduced CCRBA call followed by indexed body/site `ComputeJacobians` to match the LeoQuad runtime order. The lighter `ComputeDynamicsRNEA` path is closer to MuJoCo's bias-force path and avoids the CCRBA matrices. CCRBA computes gravity, Coriolis, and nonlinear torque ingredients from the centroidal/`Cmat` pass instead of calling RNEA. In the extended substage, dense output clearing is small; `Cmat` and `MmatDot` are the main per-term matrix costs, and the standalone `Cmat*qvel` row is now a diagnostic cost avoided by the public CCRBA entry point. In the prerequisite substage, zero-velocity states are now much cheaper because the local bias matrix and zero bias products are skipped exactly, `BmatCompNonzero` avoids scanning/accumulating known-zero composite bias matrices, and zero-derivative DOF bias products write directly into `f2` without a returned temporary. Nonzero velocity states still pay the real local-bias cost, but grouped local motion-cross updates reduce that row from roughly 1.5 us to roughly 0.8 us on the current real XML fixture. A readability pass restored the local momentum calculation to the dtMath `inertia * velocity` operator form after benchmarking showed no meaningful regression. Direct in-place spatial inertia filling reduces the composite inertia row to about 0.5 us, and in-place scaled adds plus direct `cdof`/`cdof_dot` spatial writes remove small temporaries from the prerequisite path. Fixed-size spatial matrix-vector kernels in the DOF bias-force path are unrolled because those products are always 6x6. Jacobian computation is separated from dynamics computation. The indexed Jacobian overload computes the requested root/base body Jacobian plus task-site Jacobians only; for the quadruped fixture this reduces Jacobian median time from about 3.7 us to about 1.05-1.07 us without counted C++ `operator new` allocation; the raw CSV retains the zero allocation counters. Public `ComputePointJacobianTerms`, `ComputeBodyJacobianTerms`, and `ComputeSiteJacobianTerms` wrappers cover one-off task requests that need `J`, `Jdot`, and `Jdot*qvel` together. Site-only Jacobian timing is useful as a diagnostic lower bound, but it is not the LeoQuad runtime contract while the base body Jacobian remains a controller input.

## Reference Paths

MuJoCo scope: `mj_forward`, `mj_subtreeVel`, `mj_fullM`, site Jacobians, site `Jdot`, subtree CoM Jacobian, and angular momentum matrix.

Pinocchio scope: `crba`, generalized gravity, nonlinear effects, Coriolis matrix, `ccrba`, centroidal map time variation, and centroidal momentum time variation.

| Engine | Fixture | State | nq | nv | bodies | mean us | median us | p99 us |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| mujoco | single_manipulator_fixed_base | nominal | 3 | 3 | 5 | 1.311790 | 1.309000 | 1.370000 |
| mujoco | single_manipulator_fixed_base | asymmetric | 3 | 3 | 5 | 1.421240 | 1.419000 | 1.486000 |
| mujoco | single_manipulator_fixed_base | nonzero_velocity | 3 | 3 | 5 | 1.417905 | 1.415000 | 1.498000 |
| mujoco | dual_manipulator_fixed_base | nominal | 4 | 4 | 6 | 1.676770 | 1.583000 | 2.766000 |
| mujoco | dual_manipulator_fixed_base | asymmetric | 4 | 4 | 6 | 1.733755 | 1.668000 | 3.002000 |
| mujoco | dual_manipulator_fixed_base | nonzero_velocity | 4 | 4 | 6 | 1.724555 | 1.668000 | 2.177000 |
| mujoco | multi_robot_fixed_base | nominal | 4 | 4 | 8 | 2.082535 | 2.035000 | 2.569000 |
| mujoco | multi_robot_fixed_base | asymmetric | 4 | 4 | 8 | 2.182065 | 2.179000 | 2.264000 |
| mujoco | multi_robot_fixed_base | nonzero_velocity | 4 | 4 | 8 | 2.178980 | 2.146000 | 2.374000 |
| mujoco | multi_robot_floating_base | nominal | 18 | 16 | 8 | 3.527945 | 3.433000 | 5.424000 |
| mujoco | multi_robot_floating_base | asymmetric | 18 | 16 | 8 | 3.671085 | 3.669000 | 3.769000 |
| mujoco | multi_robot_floating_base | nonzero_velocity | 18 | 16 | 8 | 3.773460 | 3.730000 | 4.089000 |
| mujoco | quadruped_floating_base | nominal | 19 | 18 | 14 | 5.660370 | 5.442000 | 7.967000 |
| mujoco | quadruped_floating_base | asymmetric | 19 | 18 | 14 | 6.102995 | 6.060000 | 6.381000 |
| mujoco | quadruped_floating_base | nonzero_velocity | 19 | 18 | 14 | 6.195770 | 6.101000 | 9.170000 |
| mujoco | fixed_body_heavy | nominal | 1 | 1 | 11 | 1.922715 | 1.921000 | 2.003000 |
| mujoco | fixed_body_heavy | asymmetric | 1 | 1 | 11 | 2.003010 | 1.958000 | 2.311000 |
| mujoco | fixed_body_heavy | nonzero_velocity | 1 | 1 | 11 | 1.991585 | 1.962000 | 3.391000 |
| pinocchio | single_manipulator_fixed_base | nominal | 3 | 3 | 5 | 3.219100 | 3.130000 | 4.212000 |
| pinocchio | single_manipulator_fixed_base | asymmetric | 3 | 3 | 5 | 3.356900 | 3.213000 | 4.364000 |
| pinocchio | single_manipulator_fixed_base | nonzero_velocity | 3 | 3 | 5 | 3.321410 | 3.222000 | 4.176000 |
| pinocchio | dual_manipulator_fixed_base | nominal | 4 | 4 | 6 | 4.207190 | 4.022000 | 5.095000 |
| pinocchio | dual_manipulator_fixed_base | asymmetric | 4 | 4 | 6 | 4.169210 | 4.117000 | 5.400000 |
| pinocchio | dual_manipulator_fixed_base | nonzero_velocity | 4 | 4 | 6 | 4.186035 | 4.120000 | 5.305000 |
| pinocchio | multi_robot_fixed_base | nominal | 4 | 4 | 8 | 4.088515 | 4.002000 | 5.257000 |
| pinocchio | multi_robot_fixed_base | asymmetric | 4 | 4 | 8 | 4.348015 | 4.210000 | 8.888000 |
| pinocchio | multi_robot_fixed_base | nonzero_velocity | 4 | 4 | 8 | 4.246310 | 4.123000 | 5.609000 |
| pinocchio | multi_robot_floating_base | nominal | 18 | 16 | 8 | 7.783510 | 7.597000 | 13.166000 |
| pinocchio | multi_robot_floating_base | asymmetric | 18 | 16 | 8 | 8.342935 | 7.725000 | 15.300000 |
| pinocchio | multi_robot_floating_base | nonzero_velocity | 18 | 16 | 8 | 7.804655 | 7.736000 | 13.424000 |
| pinocchio | quadruped_floating_base | nominal | 19 | 18 | 14 | 14.882575 | 14.768000 | 22.538000 |
| pinocchio | quadruped_floating_base | asymmetric | 19 | 18 | 14 | 15.159605 | 15.086000 | 20.512000 |
| pinocchio | quadruped_floating_base | nonzero_velocity | 19 | 18 | 14 | 15.214840 | 15.138000 | 20.843000 |
| pinocchio | fixed_body_heavy | nominal | 1 | 1 | 11 | 1.035020 | 1.032000 | 1.086000 |
| pinocchio | fixed_body_heavy | asymmetric | 1 | 1 | 11 | 1.061915 | 1.056000 | 1.126000 |
| pinocchio | fixed_body_heavy | nonzero_velocity | 1 | 1 | 11 | 1.067065 | 1.054000 | 1.185000 |

## Reference Error Summary

`benchmark_dtDynamicsReferences` now emits numerical error rows after the timing table. Rows with `status=ok` are numerical comparisons. Rows with `status=skipped` are explicit diagnostics for intentionally non-comparable quantities.

MuJoCo comparisons cover:

- `Mmat`
- `gravityTorq`, `coriolisTorq`, `nonlinTorq`
- `cdof`, `cdof_dot`, `cvel`
- site `jacp`, `jacr`, `jacp_dot`, `jacr_dot`
- `subtree_com`, `comJmat`, `cmAmat`, `centMom`

All MuJoCo max/RMS errors were `0.000000` at benchmark output precision for every fixture/state in this run. The `multi_robot_fixed_base` fixture adds two independent world-root bodies, a fixed sensor frame, multiple sites, limited joints, armatures, and actuators. The `multi_robot_floating_base` fixture adds two independent free-joint roots with separate articulated branches. The MuJoCo model-parity test also checks `nroot`, `root_bodyid`, `root_mass`, and per-body `body_rootidx` for these multi-root cases.

Pinocchio comparisons cover:

- `Mmat`
- `gravityTorq`, `coriolisTorq`, `nonlinTorq`
- floating-base quadruped `cmAmat`, `centMom`, and `cmAmatDot*qvel`
- floating-base multi-robot `Mmat`, `gravityTorq`, `coriolisTorq`, and `nonlinTorq`

All Pinocchio max/RMS errors were `0.000000` at benchmark output precision for `status=ok` rows. The fixed-base multi-robot Pinocchio reference includes matching armature values so its mass matrix is comparable to the MuJoCo/native fixture. The multi-floating Pinocchio reference now uses explicit free-joint qpos/qvel maps for both independent roots. Its whole-system mass, gravity, nonlinear, and Coriolis terms are comparable to native `dtDynamics`; multi-root centroidal rows are emitted as `status=skipped` because native `dtDynamics` stores those outputs per-root stacked, while Pinocchio exposes whole-system centroidal quantities for the hand-built reference model. Pinocchio centroidal rows for fixed-base benchmark fixtures are also emitted as `status=skipped` because the benchmark XML includes world-fixed inertial bodies; those bodies affect native/MuJoCo whole-subtree centroidal references but are not represented in the current hand-built fixed-base Pinocchio models.

## Current Interpretation

The native implementation has no counted C++ `operator new` allocation in the documented realtime dynamics benchmark rows for the tested fixtures. The synthetic floating-base quadruped remains a useful package-local proxy for the LeoQuad WBC path, and the opt-in external XML fixture records the real `LeoQuad_QD1.xml` model without coupling the standalone package to LeoQuad. The timing tables above are evidence snapshots from the stated benchmark/profile runs; compare future work against freshly regenerated CSVs and logs rather than mixing historic representative values. In the recorded real-XML combined reduced-CCRBA-plus-indexed-Jacobian snapshot, batch-summary medians were 6.210/6.503 us for nominal/asymmetric zero-velocity states and 7.830 us for the nonzero-velocity state. In the recorded isolated-prefix installed-package headless LeoQuad profile snapshot, `Robot::UpdateDynamics` mean runtime was 15.589 us, including 12.663 us native mean split into 10.232 us CCRBA and 2.432 us indexed Jacobians, plus 2.794 us controller-facing export.

A focused allocation-instrumentation rerun of `benchmark_dtDynamics` keeps allocation counters in the CSV for regression checks. Human-facing runtime tables omit all-zero compute allocation columns; the diagnostic setup allocation counts remain summarized below only to explain initialization resource behavior:

| Fixture | nq | nv | bodies | resize alloc mean | resize alloc max | resize bytes mean | resize bytes max |
|---|---:|---:|---:|---:|---:|---:|---:|
| single_manipulator_fixed_base | 3 | 3 | 5 | 31 | 31 | 8900 | 8900 |
| dual_manipulator_fixed_base | 4 | 4 | 6 | 31 | 31 | 10936 | 10936 |
| multi_robot_fixed_base | 4 | 4 | 8 | 31 | 31 | 14112 | 14112 |
| multi_robot_floating_base | 18 | 16 | 8 | 31 | 31 | 17600 | 17600 |
| quadruped_floating_base | 19 | 18 | 14 | 31 | 31 | 27720 | 27720 |
| fixed_body_heavy | 1 | 1 | 11 | 31 | 31 | 17116 | 17116 |
| serial_chain_10 | 10 | 10 | 12 | 31 | 31 | 21696 | 21696 |
| serial_chain_30 | 30 | 30 | 32 | 31 | 31 | 58256 | 58256 |

Interpretation: zero allocation columns are correct for the measured steady-state compute scope under the benchmark C++ allocation counter, not for setup and not as a complete heap-allocation proof. In the resource table, `resize alloc mean/max` are allocation-call counts, not microseconds; `resize bytes mean/max` are requested bytes; only the timing columns are in microseconds. Mean and max are equal in the current resource rows because every fresh `dtDynData` resize for a fixed model takes the same counted allocation path. The benchmark counters cover C++ global allocation operators in the executable; dtMath dynamic containers and external libraries may allocate through direct `malloc`/`calloc`/`realloc` paths that require a malloc-family interposer, fail-on-allocation guard, sanitizer, or OS heap/profiler tooling for certification.

The MuJoCo reference path is still faster for the overlapping reference scope, but the Release comparison is about 2.4-2.9x on the floating-base quadruped median path, not the roughly 30x ratio implied by an unoptimized native build. MuJoCo also does not compute all CCRBA outputs in the same form. The Pinocchio reference path is close to native for the quadruped under this call sequence and currently serves mainly as an analytic centroidal/dynamics validation reference.

The p99 values are still noisy because the benchmark is a simple process-local timer without CPU isolation or realtime priority. Use `DTDYNAMICS_BENCHMARK_BATCHES=N` to record repeated batches and prefer the batch-summary median of per-batch means/medians when comparing small changes.

## Benchmark Completion Status

The benchmark and validation scope is complete for the current redesign phase: the native benchmarks, stage benchmarks, MuJoCo 3.8.0 reference timing, Pinocchio 3.9.0 reference timing, repeated-batch metadata, multi-robot fixed/floating fixtures, derivative-like quantity figures, and LeoQuad headless runtime timing are implemented and documented.

The remaining nonzero-velocity local-bias matrix row is now an optional future optimization target, not a release blocker. Reducing it further would likely require replacing readable `ComputeBiasMatrix` assembly helpers with a larger hand-expanded formula. That tradeoff should be treated as a separate design decision because the current implementation keeps zero counted C++ allocation in the documented steady-state compute benchmarks, passes MuJoCo/Pinocchio validation, and preserves source readability.
