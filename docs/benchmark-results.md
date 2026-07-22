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
build/validation/bin/benchmark_dtDynamics > /tmp/native.csv
build/validation/bin/benchmark_dtDynamicsStages > /tmp/stages.csv
build/validation/bin/benchmark_dtDynamicsReferences > /tmp/references.csv
python3 tools/plot_benchmark_results.py \
  --native /tmp/native.csv \
  --stages /tmp/stages.csv \
  --references /tmp/references.csv \
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

## Part A — Native dynamics and reference scope

Part A contains five timing paths. The first three are package-native dynamics-only calls; the last two are external reference call sequences:

- `native_rnea_com_pos`: `ComputeDynamicsRNEA(model, data, gravity, DTDYN_RNEA_TERM_COM_POS)`.
- `native_ccrba_reduced`: `ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_COM_POS | DTDYN_CCRBA_TERM_CMAMAT | DTDYN_CCRBA_TERM_CENT_MOM)`.
- `native_ccrba_all`: `ComputeDynamicsCCRBA(model, data, gravity, DTDYN_CCRBA_TERM_ALL)`.
- `mujoco`: the documented MuJoCo forward/dynamics/reference sequence.
- `pinocchio`: the documented Pinocchio dynamics/centroidal reference sequence.

MuJoCo and Pinocchio do not expose an identical CCRBA output contract. Their rows are contextual reference timings only, not same-output performance comparisons. Numerical comparability is handled separately by the status-bearing reference-error rows.

The native CSV `scope` column prevents the three package-native calls from being mixed. It contains 72 native timing rows: eight fixtures, three states, and three scopes. RNEA reports centroidal-matrix and CoM-velocity consistency as `nan`; reduced CCRBA validates `centMom = cmAmat * qvel` but reports CoM-velocity consistency as `nan`; full CCRBA validates both centroidal momentum and `comVel = comJmat * qvel`.

### Nominal Part A timing

| Fixture | RNEA + CoM median / p99 us | Reduced CCRBA median / p99 us | Full CCRBA median / p99 us | MuJoCo median / p99 us | Pinocchio median / p99 us |
|---|---:|---:|---:|---:|---:|
| single_manipulator_fixed_base | 1.504 / 1.573 | 1.084 / 1.156 | 1.232 / 1.318 | 1.182 / 2.404 | 2.496 / 2.556 |
| dual_manipulator_fixed_base | 1.881 / 1.967 | 1.323 / 1.410 | 1.497 / 1.600 | 1.274 / 1.322 | 3.205 / 4.157 |
| multi_robot_fixed_base | 2.265 / 2.347 | 1.538 / 1.681 | 1.739 / 1.839 | 1.631 / 1.704 | 3.219 / 4.413 |
| multi_robot_floating_base | 3.125 / 3.331 | 2.936 / 3.059 | 13.674 / 26.841 | 3.110 / 5.945 | 6.055 / 6.268 |
| quadruped_floating_base | 5.809 / 6.027 | 5.032 / 5.604 | 6.365 / 6.609 | 4.888 / 5.090 | 13.229 / 73.571 |
| fixed_body_heavy | 2.826 / 2.922 | 1.592 / 1.691 | 1.658 / 1.724 | 1.588 / 1.782 | 1.266 / 1.336 |
| serial_chain_10 | 4.375 / 4.582 | 3.513 / 3.638 | 4.249 / 4.398 | 3.256 / 3.587 | 12.962 / 16.006 |
| serial_chain_30 | 13.963 / 18.645 | 23.695 / 27.705 | 18.191 / 25.845 | 15.246 / 20.411 | 41.240 / 44.983 |

### Floating-base quadruped states

| State | RNEA + CoM median / p99 us | Reduced CCRBA median / p99 us | Full CCRBA median / p99 us | MuJoCo median / p99 us | Pinocchio median / p99 us |
|---|---:|---:|---:|---:|---:|
| nominal | 5.809 / 6.027 | 5.032 / 5.604 | 6.365 / 6.609 | 4.888 / 5.090 | 13.229 / 73.571 |
| asymmetric | 5.878 / 6.085 | 5.175 / 5.427 | 6.492 / 6.778 | 4.938 / 5.523 | 18.417 / 20.137 |
| nonzero_velocity | 6.021 / 6.216 | 6.948 / 7.176 | 8.221 / 8.515 | 4.943 / 5.280 | 18.442 / 20.768 |

For all three native scopes, mass-matrix symmetry and `nonlinTorq = coriolisTorq + gravityTorq` consistency errors were zero at benchmark output precision. Reduced/full CCRBA additionally reported zero centroidal-momentum consistency error; full CCRBA also reported zero CoM-velocity consistency error.

### Figures

The runtime and scaling figures include all five Part A entries. All figures are generated from the same CSV outputs as the tables:

![Nominal runtime by fixture](figures/runtime_by_fixture.png)

![Runtime scaling by velocity dimension](figures/scaling_by_dof.png)

![Nominal stage breakdown](figures/stage_breakdown.png)

![Reference error summary](figures/reference_error_summary.png)

![Quadruped nonzero-velocity latency distribution](figures/latency_cdf.png)

### Native stage breakdown

`benchmark_dtDynamicsStages` splits the native pipeline into public API stages and internal CCRBA/RNEA substages. Each stage starts from the prerequisite state needed by that API, so the rows are diagnostic timings rather than additive exclusive profiler samples.

The setup-only `benchmark_dtDynamics` resource rows (`dtDynModel_resource` and `dtDynData_resource`) inspect model compilation and data resize cost outside the realtime loop. These rows are included here to make setup cost explicit; they are not part of the controller update timing. The human-facing table omits allocation columns because compute allocation columns are all zero and setup allocation counts are diagnostic counter values rather than time. The CSV still keeps allocation counters for regression checks.

| Fixture | nq | nv | bodies | Compile median us | Compile p99 us | Data resize median us | Data resize p99 us |
|---|---:|---:|---:|---:|---:|---:|---:|
| single_manipulator_fixed_base | 3 | 3 | 5 | 1.811 | 3.263 | 1.858 | 2.696 |
| dual_manipulator_fixed_base | 4 | 4 | 6 | 2.205 | 2.342 | 2.312 | 4.180 |
| multi_robot_fixed_base | 4 | 4 | 8 | 2.807 | 4.197 | 2.470 | 3.467 |
| multi_robot_floating_base | 18 | 16 | 8 | 3.300 | 4.353 | 3.811 | 5.956 |
| quadruped_floating_base | 19 | 18 | 14 | 6.824 | 8.136 | 5.558 | 7.859 |
| fixed_body_heavy | 1 | 1 | 11 | 2.664 | 3.826 | 2.714 | 4.043 |
| serial_chain_10 | 10 | 10 | 12 | 3.606 | 4.550 | 3.841 | 5.910 |
| serial_chain_30 | 30 | 30 | 32 | 12.691 | 15.620 | 10.753 | 12.607 |

For the floating-base quadruped fixture, the median stage times in microseconds were:

| State | FK | Mass after FK | CCRBA after FK | CCRBA prereq | CCRBA extended | Body Jacobians | Site Jacobians | All Jacobians | Indexed Jacobians | Gravity | RNEA + CoM | CCRBA all |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| nominal | 2.003 | 1.564 | 4.669 | 1.128 | 3.336 | n/a | n/a | 6.951 | 1.963 | 0.045 | 4.207 | 4.637 |
| asymmetric | 1.551 | 1.153 | 3.119 | 0.742 | 2.439 | n/a | n/a | 2.524 | 0.688 | 0.045 | 4.124 | 5.086 |
| nonzero_velocity | 1.634 | 1.226 | 4.767 | 1.990 | 2.697 | n/a | n/a | 2.984 | 0.786 | 0.050 | 4.486 | 6.011 |

A focused run for the multi-root fixtures validates the timing path for independent model parts plus fixed sensor frames. Median stage times in microseconds were:

| Fixture | State | FK | Mass after FK | CCRBA after FK | CCRBA prereq | CCRBA extended | Indexed Jacobians | RNEA + CoM | CCRBA all | Reduced WBC update |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| multi_robot_fixed_base | nominal | 0.631 | 0.420 | 0.689 | 0.381 | 0.274 | 0.091 | 1.649 | 1.253 | 1.229 |
| multi_robot_fixed_base | asymmetric | 0.641 | 0.408 | 0.694 | 0.373 | 0.275 | 0.082 | 1.530 | 1.192 | 1.157 |
| multi_robot_fixed_base | nonzero_velocity | 0.585 | 0.371 | 0.947 | 0.656 | 0.236 | 0.082 | 1.549 | 1.494 | 1.458 |
| multi_robot_floating_base | nominal | 0.649 | 0.790 | 2.663 | 0.571 | 2.031 | 0.542 | 2.401 | 2.983 | 2.917 |
| multi_robot_floating_base | asymmetric | 0.758 | 1.160 | 10.355 | 2.153 | 3.754 | 0.823 | 3.822 | 5.192 | 5.469 |
| multi_robot_floating_base | nonzero_velocity | 0.949 | 1.102 | 4.529 | 1.627 | 2.507 | 0.687 | 3.137 | 4.789 | 4.789 |

## Part B — LeoQuad controller scope

Part B applies the same reduced CCRBA mask defined in Part A to the canonical LeoQuad model, then adds the controller-selected indexed Jacobians:

```cpp
constexpr dtDynCcrbaTermFlags leoquadMask =
    DTDYN_CCRBA_TERM_COM_POS |
    DTDYN_CCRBA_TERM_CMAMAT |
    DTDYN_CCRBA_TERM_CENT_MOM;

ComputeDynamicsCCRBA(model, data, gravity, leoquadMask);
ComputeJacobians(model, data, bodyIds, siteIds);
```

`bodyIds` contains the root/base body and `siteIds` contains the four foot sites. The dynamics-only reduced call is labeled `leoquad_reduced_ccrba`; the combined controller-shaped call is `leoquad_wbc_update`. The following fresh 200-iteration medians use the canonical current `leoquad-control/model/LeoQuad_QD1.xml` fixture and are evidence snapshots rather than platform-independent guarantees:

| State | CCRBA all median us | RNEA + CoM median us | Reduced CCRBA median us | Reduced CCRBA + indexed Jacobians median us |
|---|---:|---:|---:|---:|
| nominal | 6.392 | 5.942 | 5.128 | 7.673 |
| asymmetric | 6.646 | 5.943 | 5.373 | 9.096 |
| nonzero_velocity | 8.053 | 6.016 | 6.884 | 9.416 |

The external fixture remains opt-in through `DTDYNAMICS_BENCHMARK_XML`; `DTDYNAMICS_BENCHMARK_ONLY_EXTERNAL=1` restricts the run to that XML and `DTDYNAMICS_BENCHMARK_XML_NAME` controls the CSV fixture label. The standalone package therefore does not depend on LeoQuad assets.

After replacing temporary 6x6 cross-matrix construction in the CCRBA bias-force assembly with direct sparse cross-term accumulation, removing the redundant local `Bmat` scratch array, computing local bias directly into `BmatComp`, fusing the public CCRBA `Cmat*qvel` product into the `Cmat` fill loop, skipping zero-velocity local bias construction, skipping zero bias-force products, avoiding redundant nonzero-body inertia clears, grouping local motion-cross updates by spatial-inertia column, combining paired DOF 6x6-vector products, filling body spatial inertia directly in place, replacing scaled spatial temporaries with in-place scaled adds, caching compiled body/inertial/site rotation matrices, using a data-internal combined indexed `J`/`Jdot` writer for controller-selected body/site Jacobians, moving diff-motion zeroing to exact write sites with exact-zero cross-product skips, writing `cdof` and nonzero `cdof_dot` directly, keeping the local momentum multiply inside `ComputeBiasMatrix` in dtMath operator form after benchmark-confirming no meaningful regression, carrying a per-body `BmatCompNonzero` flag through composite bias accumulation, skipping redundant `BmatComp` clears when the flag prevents reads, writing zero-derivative DOF bias products directly into `f2` with `SpatialMulInto`, and unrolling fixed-size 6x6 spatial matrix-vector products in the DOF bias-force path, the following historical optimization batch-summary medians were recorded before the fresh scope run above:

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

| State | Dense clear | Root CoM | CMM/CMMdot | Mmat | MmatDot | Cmat | CoM J/Jdot | CMM*qvel | CoM J*qvel | Cmat*qvel | Combined products |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| nominal | 0.105 | 0.023 | 0.148 | 0.347 | 0.598 | 0.717 | 0.170 | 0.207 | 0.131 | 0.533 | 0.817 |
| asymmetric | 0.115 | 0.024 | 0.150 | 0.344 | 0.618 | 0.689 | 0.169 | 0.203 | 0.126 | 0.501 | 0.775 |
| nonzero_velocity | 0.105 | 0.023 | 0.146 | 0.346 | 0.597 | 0.689 | 0.169 | 0.202 | 0.126 | 0.500 | 0.776 |

`Combined products` is one timed stage that executes the three matrix-vector products sequentially: `centMom = cmAmat * qvel`, `comVel = comJmat * qvel`, and the standalone diagnostic `coriolisTorq = Cmat * qvel`. It is not a fourth mathematical output, and its measured median need not equal the arithmetic sum of the three separately timed medians because each row has independent timer/cache effects.

The `Cmat*qvel` row remains in the stage benchmark as a diagnostic standalone product. The public `ComputeDynamicsCCRBA` path no longer pays that separate dense pass: it accumulates `coriolisTorq` while filling `Cmat`, then forms `nonlinTorq = gravityTorq + coriolisTorq`.

The old top-level spatial-array pre-clear was removed after confirming downstream stages overwrite the spatial work arrays (`cdof`, `cdof_dot`, `cvel`, `cinert`, `crb`, `BmatComp`, and `f1/f2/f3`). Dense output matrix clearing remains inside the CCRBA path because non-ancestor entries in `Mmat`, `MmatDot`, and `Cmat` are intentionally not written by the parent-chain loops and must remain zero.

Fresh CCRBA-all nominal medians were 1.218 us for the single manipulator, 1.509 us for the dual manipulator, 1.739 us for multi-robot fixed base, 3.854 us for multi-robot floating base, and 1.609 us for the fixed-body-heavy fixture. All measured stage compute rows reported zero counted C++ `operator new` allocation; the raw stage CSV retains allocation counters for regression checks.

This explains most of the remaining native-vs-MuJoCo timing difference. `DTDYN_CCRBA_TERM_ALL` computes controller-specific quantities beyond the MuJoCo overlap set. The package-native `native_ccrba_reduced` scope requests only `COM_POS`, `CMAMAT`, and `CENT_MOM`. Part B applies the same mask to the canonical LeoQuad model as `leoquad_reduced_ccrba`. The stage benchmark also includes `leoquad_wbc_update`, which runs the reduced CCRBA call followed by indexed body/site `ComputeJacobians` to match the LeoQuad runtime order. The native `compute_dynamics_rnea_com_pos` stage uses `DTDYN_RNEA_TERM_COM_POS` and is closer to MuJoCo's bias-force path and avoids the CCRBA matrices. CCRBA computes gravity, Coriolis, and nonlinear torque ingredients from the centroidal/`Cmat` pass instead of calling RNEA. In the extended substage, dense output clearing is small; `Cmat` and `MmatDot` are the main per-term matrix costs, and the standalone `Cmat*qvel` row is now a diagnostic cost avoided by the public CCRBA entry point. In the prerequisite substage, zero-velocity states are now much cheaper because the local bias matrix and zero bias products are skipped exactly, `BmatCompNonzero` avoids scanning/accumulating known-zero composite bias matrices, and zero-derivative DOF bias products write directly into `f2` without a returned temporary. Nonzero velocity states still pay the real local-bias cost, but grouped local motion-cross updates reduce that row from roughly 1.5 us to roughly 0.8 us on the current real XML fixture. A readability pass restored the local momentum calculation to the dtMath `inertia * velocity` operator form after benchmarking showed no meaningful regression. Direct in-place spatial inertia filling reduces the composite inertia row to about 0.5 us, and in-place scaled adds plus direct `cdof`/`cdof_dot` spatial writes remove small temporaries from the prerequisite path. Fixed-size spatial matrix-vector kernels in the DOF bias-force path are unrolled because those products are always 6x6. Jacobian computation is separated from dynamics computation. The indexed Jacobian overload computes the requested root/base body Jacobian plus task-site Jacobians only; for the quadruped fixture this reduces Jacobian median time from about 3.7 us to about 1.05-1.07 us without counted C++ `operator new` allocation; the raw CSV retains the zero allocation counters. Public `ComputePointJacobianTerms`, `ComputeBodyJacobianTerms`, and `ComputeSiteJacobianTerms` wrappers cover one-off task requests that need `J`, `Jdot`, and `Jdot*qvel` together. Site-only Jacobian timing is useful as a diagnostic lower bound, but it is not the LeoQuad runtime contract while the base body Jacobian remains a controller input.

## Reference and validation scope

MuJoCo scope: `mj_forward`, `mj_subtreeVel`, `mj_fullM`, site Jacobians, site `Jdot`, subtree CoM Jacobian, and angular momentum matrix.

Pinocchio scope: `crba`, generalized gravity, nonlinear effects, Coriolis matrix, `ccrba`, centroidal map time variation, and centroidal momentum time variation.

| Engine | Fixture | State | nq | nv | bodies | mean us | median us | p99 us |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| mujoco | single_manipulator_fixed_base | nominal | 3 | 3 | 5 | 1.352030 | 1.182000 | 2.404000 |
| mujoco | single_manipulator_fixed_base | asymmetric | 3 | 3 | 5 | 1.263435 | 1.261000 | 1.326000 |
| mujoco | single_manipulator_fixed_base | nonzero_velocity | 3 | 3 | 5 | 1.199990 | 1.152000 | 1.461000 |
| mujoco | dual_manipulator_fixed_base | nominal | 4 | 4 | 6 | 1.274930 | 1.274000 | 1.322000 |
| mujoco | dual_manipulator_fixed_base | asymmetric | 4 | 4 | 6 | 1.408185 | 1.365000 | 1.419000 |
| mujoco | dual_manipulator_fixed_base | nonzero_velocity | 4 | 4 | 6 | 1.364265 | 1.361000 | 1.423000 |
| mujoco | multi_robot_fixed_base | nominal | 4 | 4 | 8 | 1.661830 | 1.631000 | 1.704000 |
| mujoco | multi_robot_fixed_base | asymmetric | 4 | 4 | 8 | 1.729135 | 1.728000 | 1.779000 |
| mujoco | multi_robot_fixed_base | nonzero_velocity | 4 | 4 | 8 | 1.943250 | 1.913000 | 1.998000 |
| mujoco | multi_robot_floating_base | nominal | 18 | 16 | 8 | 3.899355 | 3.110000 | 5.945000 |
| mujoco | multi_robot_floating_base | asymmetric | 18 | 16 | 8 | 3.370955 | 3.330000 | 3.391000 |
| mujoco | multi_robot_floating_base | nonzero_velocity | 18 | 16 | 8 | 3.405825 | 3.380000 | 3.470000 |
| mujoco | quadruped_floating_base | nominal | 19 | 18 | 14 | 4.928445 | 4.888000 | 5.090000 |
| mujoco | quadruped_floating_base | asymmetric | 19 | 18 | 14 | 5.001920 | 4.938000 | 5.523000 |
| mujoco | quadruped_floating_base | nonzero_velocity | 19 | 18 | 14 | 4.990995 | 4.943000 | 5.280000 |
| mujoco | fixed_body_heavy | nominal | 1 | 1 | 11 | 1.630985 | 1.588000 | 1.782000 |
| mujoco | fixed_body_heavy | asymmetric | 1 | 1 | 11 | 1.734105 | 1.733000 | 1.792000 |
| mujoco | fixed_body_heavy | nonzero_velocity | 1 | 1 | 11 | 1.726350 | 1.724000 | 1.775000 |
| mujoco | serial_chain_10 | nominal | 10 | 10 | 12 | 3.360000 | 3.256000 | 3.587000 |
| mujoco | serial_chain_10 | asymmetric | 10 | 10 | 12 | 3.941805 | 3.852000 | 4.150000 |
| mujoco | serial_chain_10 | nonzero_velocity | 10 | 10 | 12 | 4.124335 | 4.077000 | 4.196000 |
| mujoco | serial_chain_30 | nominal | 30 | 30 | 32 | 15.476185 | 15.246000 | 20.411000 |
| mujoco | serial_chain_30 | asymmetric | 30 | 30 | 32 | 16.699300 | 17.096000 | 17.592000 |
| mujoco | serial_chain_30 | nonzero_velocity | 30 | 30 | 32 | 14.031265 | 14.006000 | 15.862000 |
| pinocchio | single_manipulator_fixed_base | nominal | 3 | 3 | 5 | 2.520970 | 2.496000 | 2.556000 |
| pinocchio | single_manipulator_fixed_base | asymmetric | 3 | 3 | 5 | 2.611520 | 2.588000 | 2.685000 |
| pinocchio | single_manipulator_fixed_base | nonzero_velocity | 3 | 3 | 5 | 2.569360 | 2.565000 | 2.647000 |
| pinocchio | dual_manipulator_fixed_base | nominal | 4 | 4 | 6 | 3.285740 | 3.205000 | 4.157000 |
| pinocchio | dual_manipulator_fixed_base | asymmetric | 4 | 4 | 6 | 3.291540 | 3.287000 | 3.364000 |
| pinocchio | dual_manipulator_fixed_base | nonzero_velocity | 4 | 4 | 6 | 3.303480 | 3.291000 | 3.378000 |
| pinocchio | multi_robot_fixed_base | nominal | 4 | 4 | 8 | 3.448280 | 3.219000 | 4.413000 |
| pinocchio | multi_robot_fixed_base | asymmetric | 4 | 4 | 8 | 3.312245 | 3.300000 | 3.403000 |
| pinocchio | multi_robot_fixed_base | nonzero_velocity | 4 | 4 | 8 | 3.298635 | 3.296000 | 3.368000 |
| pinocchio | multi_robot_floating_base | nominal | 18 | 16 | 8 | 6.114680 | 6.055000 | 6.268000 |
| pinocchio | multi_robot_floating_base | asymmetric | 18 | 16 | 8 | 6.203855 | 6.179000 | 6.299000 |
| pinocchio | multi_robot_floating_base | nonzero_velocity | 18 | 16 | 8 | 6.989210 | 6.873000 | 9.269000 |
| pinocchio | quadruped_floating_base | nominal | 19 | 18 | 14 | 23.836755 | 13.229000 | 73.571000 |
| pinocchio | quadruped_floating_base | asymmetric | 19 | 18 | 14 | 18.466810 | 18.417000 | 20.137000 |
| pinocchio | quadruped_floating_base | nonzero_velocity | 19 | 18 | 14 | 18.537940 | 18.442000 | 20.768000 |
| pinocchio | fixed_body_heavy | nominal | 1 | 1 | 11 | 1.283040 | 1.266000 | 1.336000 |
| pinocchio | fixed_body_heavy | asymmetric | 1 | 1 | 11 | 1.299155 | 1.293000 | 1.368000 |
| pinocchio | fixed_body_heavy | nonzero_velocity | 1 | 1 | 11 | 1.300605 | 1.294000 | 1.362000 |
| pinocchio | serial_chain_10 | nominal | 10 | 10 | 12 | 13.008545 | 12.962000 | 16.006000 |
| pinocchio | serial_chain_10 | asymmetric | 10 | 10 | 12 | 15.037515 | 13.335000 | 24.241000 |
| pinocchio | serial_chain_10 | nonzero_velocity | 10 | 10 | 12 | 13.439960 | 13.373000 | 16.796000 |
| pinocchio | serial_chain_30 | nominal | 30 | 30 | 32 | 41.389495 | 41.240000 | 44.983000 |
| pinocchio | serial_chain_30 | asymmetric | 30 | 30 | 32 | 42.402835 | 42.207000 | 45.930000 |
| pinocchio | serial_chain_30 | nonzero_velocity | 30 | 30 | 32 | 42.415560 | 42.254000 | 45.977000 |

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

The native implementation has no counted C++ `operator new` allocation in the documented steady-state dynamics rows for the tested fixtures. The reduced CCRBA mask is now a package-native Part A scope across every fixture. Part B applies that same mask to the opt-in canonical LeoQuad XML and adds indexed Jacobians without creating an installed-package dependency. In the fresh canonical XML run, reduced-CCRBA medians were 5.128/5.373/6.884 us and reduced-CCRBA-plus-indexed-Jacobian medians were 7.673/9.096/9.416 us for nominal/asymmetric/nonzero-velocity states. The separately recorded headless controller profile is retained as historical end-to-end evidence and is not mixed into these fresh dynamics-scope numbers.

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
