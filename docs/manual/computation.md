# Computation Pipeline {#dtdynamics_computation}

Author: Joonhee Jo

@dotfile diagrams/compute_pipeline.dot

## State staging

A typical control tick writes `qpos`, `qvel`, and gravity into an already resized `dtDynData`. State staging must match the compiled model dimensions and address conventions.

## Forward kinematics

Forward kinematics computes body/site poses and motion quantities used by dynamics and Jacobian functions. Call it explicitly when only kinematics are required, or rely on dynamics entry points that perform their prerequisite kinematic updates.

## RNEA path

The manipulator equation used by the controller-facing API is

\f[
M(q)\dot{v} + h(q, v) = \tau + J(q)^\mathsf{T} f,
\f]

where `M` is the generalized mass matrix, `h` collects bias terms such as Coriolis, centrifugal, and gravity effects, `J` is a task/contact Jacobian, and `f` is an external wrench or constraint force vector supplied by higher-level software.

`ComputeDynamicsRNEA` is the lighter recursive Newton-Euler path for mass/bias/gravity/Coriolis-style ingredients available through the RNEA contract. Use it when the controller does not need CCRBA-specific centroidal/composite outputs.

## CCRBA path

The composite/centroidal path exposes quantities used in WBC objectives, including the centroidal momentum relation

\f[
h_G = A_G(q) v,
\f]

and its first-order variation

\f[
\dot{h}_G = A_G(q)\dot{v} + \dot{A}_G(q, v)v.
\f]

`ComputeDynamicsCCRBA` is the control-oriented composite rigid-body path. It computes the mass/bias basis and optional WBC terms such as `Cmat`, `MmatDot`, `cmAmat`, `cmAmatDot`, `comJmat`, `comJmatDot`, `centMom`, and CoM quantities according to term masks.

The term mask is a realtime design tool. Request only the outputs consumed by the controller tick. LeoQuad, for example, uses a reduced CCRBA mask and computes indexed body/site Jacobians separately.

## Output families and equations

The base dynamics outputs are organized around the usual split

\f[
h(q,v) = C(q,v)v + g(q),
\f]

with fields for the mass matrix `Mmat`, matrix-style Coriolis representation `Cmat`, gravity torque `gravityTorq`, Coriolis torque `coriolisTorq`, and combined nonlinear torque `nonlinTorq`. Exact controller usage varies: some stacks consume `nonlinTorq` directly, while others keep gravity and velocity terms separate for feedforward or diagnostics.

The mass matrix should be treated as a generalized-coordinate inertia matrix,

\f[
T = \frac{1}{2} v^\mathsf{T} M(q) v,
\f]

and validation checks include symmetry residuals. A controller should still decide whether it needs the full dense matrix, selected rows, or only products supplied by a higher-level layer.

Centroidal quantities are exposed per root. For a root system, the centroidal momentum matrix maps generalized velocity to centroidal momentum,

\f[
h_G = A_G(q)v,
\f]

with rows documented as `[linear; angular]`. When derivative terms are requested, the controller can form

\f[
\dot{h}_G = A_G(q)\dot{v} + \dot{A}_G(q,v)v.
\f]

CoM quantities are similarly optional. If the WBC objective only needs CoM position and Jacobian, request those flags and leave centroidal derivatives disabled.

## Dependency order

The safe mental model is:

1. state is staged in `qpos`/`qvel`;
2. forward kinematics establishes world transforms and motion subspaces;
3. dynamics fills mass, bias, gravity/Coriolis, and optional composite quantities;
4. task Jacobian calls use the current kinematic state and selected task IDs.

Dynamics entry points run their own prerequisite kinematic updates. Calling `ForwardKinematics()` explicitly is sufficient when a tick needs only transforms or a plain one-off body/site/point Jacobian. Jacobian-derivative, `Jdot*qvel`, combined-term derivative outputs, and bulk/indexed `dtDynData` Jacobian writers additionally require a current `ComputeDynamicsRNEA()` or `ComputeDynamicsCCRBA()` pass because they consume `subtree_com`, `cvel`, `cdof`, and `cdof_dot`.

## Jacobian APIs

Task velocities follow the standard differential relation

\f[
\dot{x} = J(q)v,
\f]

and acceleration-level controllers commonly consume

\f[
\ddot{x} = J(q)\dot{v} + \dot{J}(q, v)v.
\f]

Use indexed body/site Jacobian calls when the controller needs only a base body and selected task sites. Use all-body/all-site calls for diagnostics or tooling, not as the default realtime path.

For realtime WBC, prefer the combined `Compute*JacobianTerms()` helpers when a task needs both \f$J\f$ and \f$\dot{J}v\f$. Passing `nullptr` for unused outputs documents the exact ingredients needed by the task and avoids duplicate traversal work for the same body, site, or point.
