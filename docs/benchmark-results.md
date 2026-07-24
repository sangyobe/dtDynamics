# dtDynamics Benchmark Results

Date: 2026-07-23

Updated: 2026-07-23 for CPU-affined seven-batch timing, robust batch-summary reporting, refreshed validation figures, and the latest package and headless LeoQuad gates.

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
export DTDYNAMICS_BENCHMARK_CPU=0
export DTDYNAMICS_BENCHMARK_BATCHES=7
taskset -c "$DTDYNAMICS_BENCHMARK_CPU" build/validation/bin/benchmark_dtDynamics > /tmp/native.csv
taskset -c "$DTDYNAMICS_BENCHMARK_CPU" build/validation/bin/benchmark_dtDynamicsStages > /tmp/stages.csv
taskset -c "$DTDYNAMICS_BENCHMARK_CPU" build/validation/bin/benchmark_dtDynamicsReferences > /tmp/references.csv
python3 tools/plot_benchmark_results.py \
  --native /tmp/native.csv \
  --stages /tmp/stages.csv \
  --references /tmp/references.csv \
  --out docs/figures

DTDYNAMICS_BENCHMARK_ONLY_EXTERNAL=1 \
DTDYNAMICS_BENCHMARK_XML=/path/to/LeoQuad_QD1.xml \
DTDYNAMICS_BENCHMARK_XML_NAME=leoquad_qd1 \
  taskset -c "$DTDYNAMICS_BENCHMARK_CPU" \
  build/validation/bin/benchmark_dtDynamicsStages > /tmp/stages_leoquad.csv
```

Validation status for this run: `tools/audit_release_readiness.sh` passed, including `test_dtDynTypes`, `test_dtDynamicsMujocoRef`, `test_dtDynamicsPinocchioRef`, install/export validation, and release-invariant scans.

Important: benchmark numbers must come from a Release build. A default single-config CMake build leaves `CMAKE_BUILD_TYPE` empty, compiling native `dtDynamics` with no `-O` flag while MuJoCo and Pinocchio are optimized prebuilt libraries. That unoptimized configuration made native `dtDynamics` look roughly an order of magnitude slower than it is.

The Pinocchio/Boost CMake warnings observed during validation configure are not production package warnings. They come from the private Pinocchio validation prefix and CMake's Boost discovery policy/runtime-path diagnostics. `benchmark_dtDynamicsReferences` builds and runs to completion, emits both timing and error-summary CSV sections, and resolves MuJoCo/Pinocchio libraries from the standalone package's `.validation_deps` cache. These warnings should not block benchmark use unless the reference executable fails to start or reports nonzero reference errors.

All benchmark executables append a final `benchmark_metadata,key,value` CSV section. It records the executable name, build type, compiler, C++ standard, platform, CPU model, hardware concurrency, clock source, clock steadiness, warmup iterations, measured iterations, batch count, and `NDEBUG` status. This metadata is appended after timing/error rows so existing timing parsers can continue to consume the primary CSV sections by header.

Set `DTDYNAMICS_BENCHMARK_BATCHES=N` for repeated-run aggregation. When `N > 1`, the benchmarks append `benchmark_batch_summary` rows containing min/median/max of per-batch mean, median, and p99 runtime plus the maximum allocation count across batches. The plotting/report tool promotes the median of the batch means, medians, and p99 values into human tables and figures; the raw first-batch timing rows and batch min/max spread remain in the CSV. Historical or default single-batch CSVs remain unchanged.

The current timing evidence used `DTDYNAMICS_BENCHMARK_BATCHES=7` and `taskset -c 0` on an Intel Xeon w7-3545. CPU 0 affinity prevents process migration, but the host remained under the `intel_pstate` `powersave` governor without frequency locking or realtime priority. Each batch independently performs the documented warmup and measured iterations. The seven-batch central values therefore reduce one-off scheduler/DVFS distortion without claiming hard-realtime isolation.

Each batch uses 20 warmup iterations and 200 measured iterations. Allocation counts are measured by intercepting global C++ `operator new` / `operator new[]` while the measured call is active; they do not intercept dtMath or library allocations that route directly through `malloc`/`calloc`/`realloc`.

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
| single_manipulator_fixed_base | 1.196 / 1.247 | 0.849 / 0.923 | 0.964 / 1.506 | 1.165 / 1.215 | 2.793 / 2.868 |
| dual_manipulator_fixed_base | 1.501 / 1.570 | 1.046 / 1.112 | 1.187 / 1.259 | 1.368 / 1.420 | 3.575 / 3.945 |
| multi_robot_fixed_base | 1.786 / 1.856 | 1.227 / 1.286 | 1.401 / 1.450 | 1.562 / 1.619 | 3.570 / 3.671 |
| multi_robot_floating_base | 2.430 / 2.562 | 2.301 / 2.451 | 3.042 / 3.183 | 2.745 / 2.820 | 6.102 / 6.894 |
| quadruped_floating_base | 4.499 / 4.680 | 4.102 / 4.302 | 5.116 / 7.552 | 5.023 / 5.253 | 11.784 / 13.663 |
| fixed_body_heavy | 2.271 / 2.346 | 1.227 / 1.275 | 1.270 / 1.330 | 1.756 / 1.800 | 0.797 / 0.840 |
| serial_chain_10 | 3.501 / 3.719 | 2.730 / 2.853 | 3.279 / 3.425 | 3.251 / 3.344 | 9.502 / 9.684 |
| serial_chain_30 | 10.999 / 13.963 | 11.388 / 14.573 | 14.417 / 19.076 | 13.887 / 16.153 | 29.965 / 32.565 |

### Why `fixed_body_heavy` differs

`fixed_body_heavy` is a fixed-topology stress fixture, not a representative high-DOF robot. Its canonical `nbody=11`, `nq=1`, `nv=1` model contains the world, a world-fixed base, eight nested fixed mounts, and one terminal revolute link. The base has mass 4.0, each fixed mount has mass 0.05, and the terminal link has mass 1.2. The fixture tests whether an engine handles a long welded/fixed-body chain correctly even though only the final joint contributes a generalized coordinate.

Native dtDynamics and the MuJoCo fixture retain that nested body topology. The hand-built Pinocchio reference instead sums the eight fixed offsets into one placement and instantiates only the terminal revolute body; it does not traverse eight separate fixed mounts. The `nbody=11` shown for its row is the canonical fixture count used as report metadata, not Pinocchio's internal recursion count. That representation difference explains why Pinocchio is unusually fast for this fixture. Treat its timing as engine-specific fixed-body representation behavior, not as a same-morphology throughput comparison. Joint-space comparable quantities remain validation targets, while fixed-base centroidal rows that are not semantically equivalent are already emitted as `status=skipped`.

### Floating-base quadruped states

The three states are deterministic coverage conditions rather than named controller modes:

- `nominal`: model-default positions, identity free/ball-joint orientations, and zero generalized velocity.
- `asymmetric`: deterministic nonuniform position/orientation perturbations with zero generalized velocity, isolating configuration-dependent paths without velocity terms.
- `nonzero_velocity`: its own deterministic perturbed pose plus nonzero free-, ball-, and scalar-joint velocities, activating velocity-dependent Coriolis, bias, `MmatDot`, and centroidal-derivative paths. It is not the asymmetric state with velocity simply added.

Each engine adapter applies these semantics through its coordinate mapping; the contextual timing rows should not be read as one shared raw `qpos` memory layout.

| State | RNEA + CoM median / p99 us | Reduced CCRBA median / p99 us | Full CCRBA median / p99 us | MuJoCo median / p99 us | Pinocchio median / p99 us |
|---|---:|---:|---:|---:|---:|
| nominal | 4.499 / 4.680 | 4.102 / 4.302 | 5.116 / 7.552 | 5.023 / 5.253 | 11.784 / 13.663 |
| asymmetric | 4.596 / 6.409 | 4.159 / 4.327 | 5.161 / 5.359 | 5.003 / 5.715 | 13.363 / 15.723 |
| nonzero_velocity | 4.711 / 4.967 | 5.464 / 5.658 | 6.466 / 6.861 | 5.610 / 5.792 | 12.104 / 14.378 |

For all three native scopes, mass-matrix symmetry and `nonlinTorq = coriolisTorq + gravityTorq` consistency errors were zero at benchmark output precision. Reduced/full CCRBA additionally reported zero centroidal-momentum consistency error; full CCRBA also reported zero CoM-velocity consistency error. MuJoCo and Pinocchio timings remain contextual rather than same-output speed ratios because their call sequences do not compute every native CCRBA output in the same form; their primary role is numerical validation over the documented overlap set.

#### Nominal runtime by fixture

![Nominal runtime by fixture](figures/runtime_by_fixture.png)

`runtime_by_fixture.png` is a grouped-bar view of the nominal median rows in the Part A table. Each fixture group contains the three dtDynamics calls plus MuJoCo and Pinocchio. Lower bars mean less elapsed time for that specific measured call sequence on this host. The gaps among RNEA, reduced CCRBA, and full CCRBA show the cost of requesting progressively different native outputs; the MuJoCo and Pinocchio bars remain contextual because their output contracts are not identical. Tail behavior is intentionally left in the table's p99 columns rather than encoded in this median-only figure.

#### Runtime scaling by velocity dimension

![Runtime scaling by velocity dimension](figures/scaling_by_dof.png)

`scaling_by_dof.png` plots the same nominal medians against `nv`. Every marker is a different fixture, so the connecting lines are visual guides rather than fitted complexity curves. Equal `nv` does not imply equal work: floating-base topology, body count, fixed bodies, sites, and requested outputs can change runtime. Use this plot to see broad size trends, and use the fixture table for exact morphology-aware comparisons.

### Native stage breakdown

`benchmark_dtDynamicsStages` splits the native pipeline into public API stages and internal CCRBA/RNEA substages. Each stage starts from the prerequisite state needed by that API, so the rows are diagnostic timings rather than additive exclusive profiler samples.

- `CCRBA prereq` (`ccrba_prerequisites_after_fk`) assumes forward kinematics is already complete. It computes subtree CoM values, DOF motion subspaces (`cdof`), body spatial velocities and motion-subspace derivatives (`cvel`, `cdof_dot`), body/composite spatial inertias (`cinert`, `crb`), local and accumulated velocity-bias matrices, and the per-DOF `f1`/`f2`/`f3` force columns reused by downstream CCRBA outputs. Its zero-velocity checks skip bias work when `cvel` and `cdof_dot` are exactly zero.
- `CCRBA extra` (`ccrba_extended_after_prereq`) assumes those prerequisites are already complete. It clears the dense output buffers and materializes root CoM, `cmAmat`, `cmAmatDot`, `Mmat`, `MmatDot`, `Cmat`, `comJmat`, `comJmatDot`, `centMom`, `comVel`, and `coriolisTorq`. It excludes forward kinematics, the prerequisite stage itself, and the gravity/nonlinear bias-force finish of the full public call.

`CCRBA after FK` independently measures `CCRBA prereq` followed by `CCRBA extra` with FK prepared outside the timer. `CCRBA all` measures the complete public all-terms call, including its own FK and gravity/nonlinear bias-force work. Because each row is prepared and timed independently, do not add their displayed medians to reconstruct another row.

The setup-only `benchmark_dtDynamics` resource rows (`dtDynModel_resource` and `dtDynData_resource`) inspect model compilation and data resize cost outside the realtime loop. These rows are included here to make setup cost explicit; they are not part of the controller update timing. `Resize allocations` is the counted number of C++ global allocation calls during one fresh `dtDynData::Resize(model)` path; it is a count, not a time. Each of the 200 repeated resize iterations counted 31 calls, so one integer is sufficient. Requested allocation bytes remain in the raw CSV but are omitted here because they are not needed for the timing interpretation.

| Fixture | nq | nv | bodies | Compile median us | Compile p99 us | Data resize median us | Data resize p99 us | Resize allocations |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| single_manipulator_fixed_base | 3 | 3 | 5 | 1.444 | 2.401 | 1.552 | 3.090 | 31 |
| dual_manipulator_fixed_base | 4 | 4 | 6 | 1.725 | 1.819 | 1.804 | 2.359 | 31 |
| multi_robot_fixed_base | 4 | 4 | 8 | 2.202 | 2.554 | 1.988 | 3.611 | 31 |
| multi_robot_floating_base | 18 | 16 | 8 | 2.659 | 4.525 | 2.942 | 3.341 | 31 |
| quadruped_floating_base | 19 | 18 | 14 | 5.263 | 11.240 | 4.249 | 4.983 | 31 |
| fixed_body_heavy | 1 | 1 | 11 | 2.122 | 3.353 | 2.229 | 2.878 | 31 |
| serial_chain_10 | 10 | 10 | 12 | 2.879 | 4.030 | 3.219 | 5.120 | 31 |
| serial_chain_30 | 30 | 30 | 32 | 10.332 | 14.655 | 8.491 | 11.730 | 31 |

The measured steady-state dynamics and stage rows reported zero counted C++ `operator new` allocations. This distinguishes the tested compute scope from setup, but it is not a complete heap-allocation proof: dtMath containers or external libraries can use `malloc`-family paths outside this counter.

For the floating-base quadruped fixture, the median stage times in microseconds were:

| State | FK | Mass after FK | CCRBA after FK | CCRBA prereq | CCRBA extra | Body Jacobians | Site Jacobians | All Jacobians | Indexed Jacobians | Gravity | RNEA + CoM | CCRBA all |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| nominal | 1.589 | 1.272 | 3.681 | 0.886 | 2.730 | 2.035 | 0.690 | 3.035 | 0.872 | 0.054 | 4.598 | 5.126 |
| asymmetric | 1.682 | 1.318 | 3.858 | 0.908 | 2.982 | 1.971 | 0.709 | 3.081 | 0.868 | 0.054 | 4.432 | 5.053 |
| nonzero_velocity | 1.543 | 1.312 | 5.257 | 2.070 | 2.795 | 1.871 | 0.688 | 3.079 | 0.836 | 0.047 | 4.369 | 6.095 |

A focused run for the multi-root fixtures validates the timing path for independent model parts plus fixed sensor frames. Median stage times in microseconds were:

| Fixture | State | FK | Mass after FK | CCRBA after FK | CCRBA prereq | CCRBA extra | Indexed Jacobians | RNEA + CoM | CCRBA all | Reduced WBC update |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| multi_robot_fixed_base | nominal | 0.668 | 0.454 | 0.745 | 0.416 | 0.284 | 0.099 | 1.798 | 1.373 | 1.335 |
| multi_robot_fixed_base | asymmetric | 0.695 | 0.451 | 0.743 | 0.419 | 0.284 | 0.099 | 1.831 | 1.408 | 1.379 |
| multi_robot_fixed_base | nonzero_velocity | 0.702 | 0.468 | 1.138 | 0.802 | 0.284 | 0.098 | 1.867 | 1.817 | 1.783 |
| multi_robot_floating_base | nominal | 0.719 | 0.838 | 2.722 | 0.558 | 2.142 | 0.572 | 2.444 | 2.990 | 2.962 |
| multi_robot_floating_base | asymmetric | 0.752 | 0.857 | 2.891 | 0.605 | 2.166 | 0.612 | 2.495 | 3.044 | 3.059 |
| multi_robot_floating_base | nonzero_velocity | 0.748 | 1.038 | 3.878 | 1.430 | 2.185 | 0.575 | 2.578 | 3.920 | 3.908 |

#### Nominal stage breakdown figure

![Nominal stage breakdown](figures/stage_breakdown.png)

`stage_breakdown.png` compares independently timed nominal stages for the fixed-base multi-robot, floating-base multi-robot, and floating-base quadruped fixtures. It is not a stacked profile: each bar starts from the prerequisite state defined above, so the bars must not be summed to reconstruct a public call. `indexed J` measures selected Jacobians after dynamics preparation; `RNEA + CoM`, `CCRBA all`, and `reduced CCRBA` are complete dynamics calls with their stated masks; `reduced + indexed J` adds the selected body/site Jacobians after reduced CCRBA. The fixture-to-fixture gaps reflect topology and requested-work differences, not only `nv`.

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

`bodyIds` contains the root/base body and `siteIds` contains the four foot sites. The dynamics-only reduced call is labeled `leoquad_reduced_ccrba`; the combined controller-shaped call is `leoquad_wbc_update`. The following seven-batch central medians use the canonical current `leoquad-control/model/LeoQuad_QD1.xml` fixture, with 200 measured iterations per batch, and are evidence snapshots rather than platform-independent guarantees:

| State | CCRBA all median us | RNEA + CoM median us | Reduced CCRBA median us | Reduced CCRBA + indexed Jacobians median us |
|---|---:|---:|---:|---:|
| nominal | 6.383 | 5.845 | 5.162 | 7.501 |
| asymmetric | 6.529 | 5.922 | 5.255 | 7.630 |
| nonzero_velocity | 8.240 | 6.038 | 6.917 | 9.213 |

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

Fresh seven-batch central CCRBA-all nominal medians were 1.002 us for the single manipulator fixed base, 1.172 us for the dual manipulator fixed base, 1.373 us for the multi robot fixed base, 2.990 us for the multi robot floating base, and 1.169 us for `fixed_body_heavy`. All measured stage compute rows reported zero counted C++ `operator new` allocation; the raw stage CSV retains allocation counters for regression checks.

This explains most of the remaining native-vs-MuJoCo timing difference. `DTDYN_CCRBA_TERM_ALL` computes controller-specific quantities beyond the MuJoCo overlap set. The package-native `native_ccrba_reduced` scope requests only `COM_POS`, `CMAMAT`, and `CENT_MOM`. Part B applies the same mask to the canonical LeoQuad model as `leoquad_reduced_ccrba`. The stage benchmark also includes `leoquad_wbc_update`, which runs the reduced CCRBA call followed by indexed body/site `ComputeJacobians` to match the LeoQuad runtime order. The native `compute_dynamics_rnea_com_pos` stage uses `DTDYN_RNEA_TERM_COM_POS` and is closer to MuJoCo's bias-force path and avoids the CCRBA matrices. CCRBA computes gravity, Coriolis, and nonlinear torque ingredients from the centroidal/`Cmat` pass instead of calling RNEA. In the extended substage, dense output clearing is small; `Cmat` and `MmatDot` are the main per-term matrix costs, and the standalone `Cmat*qvel` row is now a diagnostic cost avoided by the public CCRBA entry point. In the prerequisite substage, zero-velocity states are now much cheaper because the local bias matrix and zero bias products are skipped exactly, `BmatCompNonzero` avoids scanning/accumulating known-zero composite bias matrices, and zero-derivative DOF bias products write directly into `f2` without a returned temporary. Nonzero velocity states still pay the real local-bias cost, but grouped local motion-cross updates reduce that row from roughly 1.5 us to roughly 0.8 us on the current real XML fixture. A readability pass restored the local momentum calculation to the dtMath `inertia * velocity` operator form after benchmarking showed no meaningful regression. Direct in-place spatial inertia filling reduces the composite inertia row to about 0.5 us, and in-place scaled adds plus direct `cdof`/`cdof_dot` spatial writes remove small temporaries from the prerequisite path. Fixed-size spatial matrix-vector kernels in the DOF bias-force path are unrolled because those products are always 6x6. Jacobian computation is separated from dynamics computation. The indexed Jacobian overload computes the requested root/base body Jacobian plus task-site Jacobians only; for the quadruped fixture this reduces Jacobian median time from about 3.7 us to about 1.05-1.07 us without counted C++ `operator new` allocation; the raw CSV retains the zero allocation counters. Public `ComputePointJacobianTerms`, `ComputeBodyJacobianTerms`, and `ComputeSiteJacobianTerms` wrappers cover one-off task requests that need `J`, `Jdot`, and `Jdot*qvel` together. Site-only Jacobian timing is useful as a diagnostic lower bound, but it is not the LeoQuad runtime contract while the base body Jacobian remains a controller input.

## Reference and validation scope

MuJoCo scope: `mj_forward`, `mj_subtreeVel`, `mj_fullM`, site Jacobians, site `Jdot`, subtree CoM Jacobian, and angular momentum matrix.

Pinocchio scope: `crba`, generalized gravity, nonlinear effects, Coriolis matrix, `ccrba`, centroidal map time variation, and centroidal momentum time variation.

| Engine | Fixture | State | nq | nv | bodies | mean us | median us | p99 us |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| mujoco | single_manipulator_fixed_base | nominal | 3 | 3 | 5 | 1.180160 | 1.165000 | 1.215000 |
| mujoco | single_manipulator_fixed_base | asymmetric | 3 | 3 | 5 | 1.294070 | 1.290000 | 1.350000 |
| mujoco | single_manipulator_fixed_base | nonzero_velocity | 3 | 3 | 5 | 1.293440 | 1.288000 | 1.342000 |
| mujoco | dual_manipulator_fixed_base | nominal | 4 | 4 | 6 | 1.371030 | 1.368000 | 1.420000 |
| mujoco | dual_manipulator_fixed_base | asymmetric | 4 | 4 | 6 | 1.462350 | 1.476000 | 1.528000 |
| mujoco | dual_manipulator_fixed_base | nonzero_velocity | 4 | 4 | 6 | 1.340780 | 1.339000 | 1.390000 |
| mujoco | multi_robot_fixed_base | nominal | 4 | 4 | 8 | 1.566050 | 1.562000 | 1.619000 |
| mujoco | multi_robot_fixed_base | asymmetric | 4 | 4 | 8 | 1.649270 | 1.632000 | 1.714000 |
| mujoco | multi_robot_fixed_base | nonzero_velocity | 4 | 4 | 8 | 1.918390 | 1.905000 | 1.973000 |
| mujoco | multi_robot_floating_base | nominal | 18 | 16 | 8 | 2.765770 | 2.745000 | 2.820000 |
| mujoco | multi_robot_floating_base | asymmetric | 18 | 16 | 8 | 3.018490 | 3.018000 | 3.108000 |
| mujoco | multi_robot_floating_base | nonzero_velocity | 18 | 16 | 8 | 3.062650 | 3.068000 | 3.209000 |
| mujoco | quadruped_floating_base | nominal | 19 | 18 | 14 | 5.107480 | 5.023000 | 5.253000 |
| mujoco | quadruped_floating_base | asymmetric | 19 | 18 | 14 | 5.143070 | 5.003000 | 5.715000 |
| mujoco | quadruped_floating_base | nonzero_velocity | 19 | 18 | 14 | 5.633740 | 5.610000 | 5.792000 |
| mujoco | fixed_body_heavy | nominal | 1 | 1 | 11 | 1.758640 | 1.756000 | 1.800000 |
| mujoco | fixed_body_heavy | asymmetric | 1 | 1 | 11 | 1.786010 | 1.784000 | 1.841000 |
| mujoco | fixed_body_heavy | nonzero_velocity | 1 | 1 | 11 | 1.826920 | 1.795000 | 1.974000 |
| mujoco | serial_chain_10 | nominal | 10 | 10 | 12 | 3.261730 | 3.251000 | 3.344000 |
| mujoco | serial_chain_10 | asymmetric | 10 | 10 | 12 | 3.620200 | 3.610000 | 3.697000 |
| mujoco | serial_chain_10 | nonzero_velocity | 10 | 10 | 12 | 3.269420 | 3.256000 | 3.393000 |
| mujoco | serial_chain_30 | nominal | 30 | 30 | 32 | 13.842700 | 13.887000 | 16.153000 |
| mujoco | serial_chain_30 | asymmetric | 30 | 30 | 32 | 15.605700 | 15.720000 | 17.560000 |
| mujoco | serial_chain_30 | nonzero_velocity | 30 | 30 | 32 | 14.523400 | 14.126000 | 16.730000 |
| pinocchio | single_manipulator_fixed_base | nominal | 3 | 3 | 5 | 2.801020 | 2.793000 | 2.868000 |
| pinocchio | single_manipulator_fixed_base | asymmetric | 3 | 3 | 5 | 2.892370 | 2.876000 | 2.960000 |
| pinocchio | single_manipulator_fixed_base | nonzero_velocity | 3 | 3 | 5 | 2.600310 | 2.597000 | 2.664000 |
| pinocchio | dual_manipulator_fixed_base | nominal | 4 | 4 | 6 | 3.591920 | 3.575000 | 3.945000 |
| pinocchio | dual_manipulator_fixed_base | asymmetric | 4 | 4 | 6 | 3.282790 | 3.295000 | 3.372000 |
| pinocchio | dual_manipulator_fixed_base | nonzero_velocity | 4 | 4 | 6 | 3.317710 | 3.309000 | 3.379000 |
| pinocchio | multi_robot_fixed_base | nominal | 4 | 4 | 8 | 3.575070 | 3.570000 | 3.671000 |
| pinocchio | multi_robot_fixed_base | asymmetric | 4 | 4 | 8 | 3.324430 | 3.315000 | 3.391000 |
| pinocchio | multi_robot_fixed_base | nonzero_velocity | 4 | 4 | 8 | 3.683860 | 3.674000 | 3.752000 |
| pinocchio | multi_robot_floating_base | nominal | 18 | 16 | 8 | 6.154000 | 6.102000 | 6.894000 |
| pinocchio | multi_robot_floating_base | asymmetric | 18 | 16 | 8 | 6.488600 | 6.359000 | 7.086000 |
| pinocchio | multi_robot_floating_base | nonzero_velocity | 18 | 16 | 8 | 6.224210 | 6.204000 | 6.307000 |
| pinocchio | quadruped_floating_base | nominal | 19 | 18 | 14 | 11.867400 | 11.784000 | 13.663000 |
| pinocchio | quadruped_floating_base | asymmetric | 19 | 18 | 14 | 13.285500 | 13.363000 | 15.723000 |
| pinocchio | quadruped_floating_base | nonzero_velocity | 19 | 18 | 14 | 12.139600 | 12.104000 | 14.378000 |
| pinocchio | fixed_body_heavy | nominal | 1 | 1 | 11 | 0.800115 | 0.797000 | 0.840000 |
| pinocchio | fixed_body_heavy | asymmetric | 1 | 1 | 11 | 0.819715 | 0.816000 | 0.865000 |
| pinocchio | fixed_body_heavy | nonzero_velocity | 1 | 1 | 11 | 0.856995 | 0.853000 | 0.911000 |
| pinocchio | serial_chain_10 | nominal | 10 | 10 | 12 | 9.168200 | 9.502000 | 9.684000 |
| pinocchio | serial_chain_10 | asymmetric | 10 | 10 | 12 | 10.660900 | 10.638000 | 11.607000 |
| pinocchio | serial_chain_10 | nonzero_velocity | 10 | 10 | 12 | 9.354540 | 9.080000 | 10.792000 |
| pinocchio | serial_chain_30 | nominal | 30 | 30 | 32 | 28.559300 | 29.965000 | 32.565000 |
| pinocchio | serial_chain_30 | asymmetric | 30 | 30 | 32 | 29.859000 | 30.710000 | 33.612000 |
| pinocchio | serial_chain_30 | nonzero_velocity | 30 | 30 | 32 | 27.799500 | 27.728000 | 30.306000 |

### Quadruped nonzero-velocity latency summary

![Quadruped nonzero-velocity latency summary](figures/latency_cdf.png)

This figure summarizes the median and p99 latency of the reference benchmark's floating-base quadruped nonzero-velocity rows. It compares the dtDynamics reference call sequence with MuJoCo and the canonical Pinocchio path; it is not the three-native-scope Part A figure. Median bars show typical measured latency, while p99 bars expose the observed tail in the CPU-affined seven-batch evidence. Affinity prevents migration but does not lock CPU frequency or provide realtime scheduling.

## Reference Error Summary

`benchmark_dtDynamicsReferences` now emits numerical error rows after the timing table. Rows with `status=ok` are numerical comparisons. Rows with `status=skipped` are explicit diagnostics for intentionally non-comparable quantities.

No reference-error figure is generated because every `status=ok` max/RMS error in this run is `0.000000` at CSV output precision. The status-bearing CSV rows and the coverage/skip explanations below carry more information than an all-zero chart.

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
