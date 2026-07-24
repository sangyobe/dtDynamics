# Controller Integration {#dtdynamics_integration}

Author: Joonhee Jo

A controller should treat dtDynamics as an ingredient engine.

## Recommended pattern

WBC consumers typically solve a task-space objective or constrained dynamics problem built from dtDynamics ingredients, for example

\f[
\min_{\dot{v}, f} \; \lVert J(q)\dot{v} + \dot{J}(q, v)v - a_{\mathrm{des}} \rVert_W^2
\f]

subject to equations of motion and task/contact constraints assembled by the controller. dtDynamics supplies the ingredients; the controller owns the optimization, torque policy, and actuator limits.

1. Compile the model once.
2. Resize and retain one `dtDynData` per realtime model instance.
3. Cache body/site IDs used by WBC tasks.
4. Each tick, update state and call the minimal dynamics/Jacobian functions needed by the controller.
5. Export only the fields consumed by the controller.

## Ingredient handoff to WBC

A typical WBC layer receives a snapshot of these ingredients:

- generalized state dimensions and cached task IDs from `dtDynModel`;
- `Mmat` and one of `nonlinTorq` or separated `gravityTorq`/`coriolisTorq`;
- task Jacobians \f$J\f$ and derivative products \f$\dot{J}v\f$ for active body/site/point tasks;
- optional CoM and centroidal quantities when the task set contains CoM or centroidal objectives.

The WBC layer remains responsible for all optimization choices. A typical WBC layer minimizes

\f[
\min_{\dot{v}, f, \tau} \; \lVert A x - b \rVert_W^2.
\f]

Subject to the dynamics constraint

\f[
M(q)\dot{v} + h(q,v) = S^\mathsf{T}\tau + J_c(q)^\mathsf{T}f.
\f]

Task, friction, actuator, and safety constraints remain owned by the controller.

dtDynamics supplies \f$M\f$, \f$h\f$, task Jacobian ingredients, and optional CoM/centroidal terms. It does not decide the support polygon, contact schedule, torque limits, or solver formulation.

## Minimal tick pseudocode

```cpp
// setup
const dtDynModel model = CompileSpec(spec);
dtDynData data;
data.Resize(model);
const dtDynIndex foot = model.SiteId("front_left_foot");

// loop
WriteEstimatorStateInto(data.qpos, data.qvel);
ComputeDynamicsCCRBA(model, data, gravity, wbcTermMask);
ComputeSiteJacobianTerms(model, data, foot, &Jp, &Jr, nullptr, nullptr, &JdvP, &JdvR);
SolveControllerProblem(data.Mmat, data.nonlinTorq, Jp, Jr, JdvP, JdvR);
```

The pseudocode deliberately leaves `WriteEstimatorStateInto()` and `SolveControllerProblem()` outside dtDynamics. They are integration-layer responsibilities.

## LeoQuad note

LeoQuad is an integration and benchmark consumer, not part of the standalone package identity. Any LeoQuad-specific adapter should stay outside the standalone package unless explicitly approved.
