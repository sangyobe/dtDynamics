# Modeling and Data Lifecycle {#dtdynamics_modeling}

Author: Joonhee Jo

The central lifecycle is:

@dotfile diagrams/model_layers.dot

## dtDynSpec: editable intent

`dtDynSpec` stores the user's intended tree model before compilation. It is the right layer for adding bodies, joints, sites, inertial data, and supported XML-subset metadata. It is not the realtime layer.

## dtDynModel: compiled structure

`dtDynModel` is produced by compiling a spec. It fixes body IDs, joint IDs, `qpos` addresses, `qvel` addresses, site IDs, root metadata, and dimensions such as `nq`, `nv`, `nbody`, `nsite`, and `nroot`.

The model should be treated as immutable during realtime control. Recompile only outside the realtime loop.

## dtDynData: preallocated runtime storage

`dtDynData` owns state vectors, output matrices, and scratch arrays sized for a specific model. Call `dtDynData::Resize(model)` before entering a hard realtime loop. The benchmark resource section measures this resize cost separately because it is intentionally not part of the zero-allocation compute path.

## Tree and coordinate convention

The compiled parent links form a tree or forest. Closed loops, parallel mechanisms, equality constraints, tendons, contact, sensors, assets, and full MJCF defaults/classes are outside the current solved-dynamics API unless a specific feature is implemented and validated. MuJoCo-compatible wording in this manual refers to indexing/convention compatibility, not a production MuJoCo dependency.

The compiled model dimensions follow the generalized-coordinate split

\f[
q \in \mathbb{R}^{n_q}, \qquad v \in \mathbb{R}^{n_v}, \qquad n_q \neq n_v
\f]

for models with quaternion floating bases. The distinction is deliberate: `qpos` stores configuration coordinates while `qvel` stores tangent-space velocities.

- Body `0` is the world body.
- Signed invalid IDs use `-1` where applicable.
- Fixed/no-joint bodies are physical bodies but do not contribute generalized velocity columns.
- Floating-base roots use the package's MuJoCo-compatible address convention for the supported coordinate type.
- Centroidal rows are per-root `[linear; angular]` rows.

## Address ownership and state vectors

Compiled addresses are the boundary between user code and the algorithms. A joint contributes a configuration interval `[qadr_j, qadr_j + nq_j)`. It also contributes a velocity interval `[vadr_j, vadr_j + nv_j)`. These are standard half-open intervals: the lower address is included and the upper address is excluded.

For scalar joints the intervals normally have the same length. For floating bases, `qpos` stores position plus orientation coordinates while `qvel` stores tangent-space spatial velocity, so the intervals differ. Write state by compiled addresses or by a higher-level adapter that was built from those addresses; do not assume body order, joint order, and velocity-column order are interchangeable.

A practical setup checklist is:

1. Build or decode `dtDynSpec`.
2. Compile it once with `CompileSpec()`.
3. Verify expected dimensions and required names (`BodyId`, `JointId`, `SiteId`).
4. Cache numeric task IDs and any state-address mapping used by the estimator/controller.
5. Resize one `dtDynData` per concurrently evaluated model instance.

## Model/data compatibility

`dtDynModel` and `dtDynData` are paired objects. A data object resized for one model should not be reused with a different model unless `Resize()` is called again. Integration code should treat `data.IsCompatible(model) == true` as a setup or debug gate before enabling a realtime loop. Recompiling a model invalidates cached assumptions about dimensions, lookup IDs, and address intervals even if the names are unchanged.

## Fixed bodies and sites

A fixed body contributes a transform and inertia but no generalized-velocity column. In address terms, a body with no joint adds no `qpos` or `qvel` interval.

Fixed bodies preserve names, inertial data, task sites, and child topology. They can matter for exported task frames even when they do not add DOFs. Do not remove or fold fixed bodies in a way that breaks public lookup or site/task indexing.
