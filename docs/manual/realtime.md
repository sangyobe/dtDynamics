# Realtime Resource Model {#dtdynamics_realtime}

Author: Joonhee Jo

@dotfile diagrams/realtime_loop.dot

Realtime users should separate setup-time allocation from control-loop computation.

## Setup phase

Perform these outside hard realtime:

1. Build or load `dtDynSpec`.
2. Compile `dtDynModel`.
3. Construct `dtDynData`.
4. Call `dtDynData::Resize(model)`.
5. Cache body/site IDs needed by the controller.

The benchmark emits a `dtDynData_resource` CSV section that measures `dtDynData` construction plus `Resize(model)` by fixture. This is the resource cost for preparing runtime storage, not the per-tick compute cost.

## Realtime phase

Inside the loop:

1. Write state into preallocated `dtDynData` vectors.
2. Call the minimal dynamics path (`ComputeDynamicsRNEA` or `ComputeDynamicsCCRBA` with a reduced term mask).
3. Compute only the body/site/point Jacobians required by the current controller.
4. Copy or reference outputs into the controller's task structures.

The benchmark CSV compute rows report `allocation_mean`, `allocation_max`, `allocation_bytes_mean`, and `allocation_bytes_max` from the executable's C++ `operator new` / `operator new[]` counter. Human-facing tables may omit these columns when all displayed compute rows are zero. For the documented benchmark paths the CSV counters should remain zero after `dtDynData::Resize(model)` and after any caller-owned destination matrices have already been sized. This is not a universal allocation guarantee for arbitrary API usage, unsized output matrices, debug logging, name lookup, dtMath malloc-family allocation paths, or operating-system/runtime behavior.

## Resource contract by phase

The intended resource split can be summarized as:

| Phase | Typical calls | Heap allocation policy |
|---|---|---|
| setup | `CompileSpec`, `dtDynData::Resize`, name lookup | allowed |
| loop | `ComputeDynamics*`, `Compute*Jacobian*` | should remain zero under the benchmark C++ allocation counter after preallocation |

A deployment can use stricter local policy, but the package documentation separates these phases so benchmark rows are interpreted correctly. `dtDynData_resource` rows are not regressions merely because they allocate; they describe setup cost. Compute rows with nonzero per-iteration allocation are the rows that need realtime investigation.

## Multi-instance controllers

Use one `dtDynData` per concurrently evaluated state. Examples include an estimator copy, an MPC rollout copy, or two controller threads evaluating the same immutable `dtDynModel`. Sharing a `dtDynModel` is appropriate; sharing a mutable `dtDynData` across simultaneous computations is not.

If the active robot description changes, re-enter setup: compile the new model, resize data, rebuild cached ID/address maps, and only then resume hard realtime computation.

## Interpreting allocation metrics

The benchmark-counter target for a preallocated compute path is

\f[
\mathrm{alloc}_{\mathrm{count}}^{\mathrm{tick}} = 0, \qquad
\mathrm{alloc}_{\mathrm{bytes}}^{\mathrm{tick}} = 0.
\f]

Setup-time resize cost is allowed outside the hard-RT loop and is reported separately. Let \f$t_i^{\mathrm{resize}}\f$ be the measured time for `dtDynData data; data.Resize(model);`. The reported resize summary uses

\f[
T_{\mathrm{resize}} = \mathrm{median}_{i=1}^{N} t_i^{\mathrm{resize}}.
\f]

Allocation count and bytes are gathered by overriding C++ allocation operators inside benchmark executables. They are diagnostic numbers, not a complete OS memory-profiler replacement: dtMath dynamic containers may allocate through `malloc`/`calloc`/`realloc`, external libraries may bypass C++ `operator new`, allocator-internal behavior is not attributed, and one-time runtime initialization outside the measured scope requires separate heap/profiler checks. They answer two RT questions:

- Does the control-path computation allocate through C++ `operator new` after preallocation?
- How much setup-time allocation does `dtDynData::Resize(model)` report through the same C++ allocation counter for this fixture size?

Use timing and allocation metrics together. A path with zero per-tick allocation can still be too slow; a setup path with allocation is acceptable if it is never called in the hard realtime loop.
