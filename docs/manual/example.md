# Minimal End-to-End Usage Example {#dtdynamics_example}

Author: Joonhee Jo

This page shows the smallest complete dtDynamics flow: build an editable tree model, compile it, preallocate runtime data, stage state, compute dynamics ingredients, and ask for one task Jacobian. The example is intentionally a dynamics-ingredient example, not a simulator or controller.

## Example program

```cpp
#include <dtDynamics/dtDynCompile.h>
#include <dtDynamics/dtDynData.h>
#include <dtDynamics/dtDynDynamics.h>
#include <dtDynamics/dtDynKinematics.h>
#include <dtDynamics/dtDynSpec.h>

#include <stdexcept>

using namespace dt::Dynamics;

namespace
{

dtDynSpec MakeTwoLinkSpec()
{
    dtDynSpec spec;

    // Body 0 is the world body created by dtDynSpec. Add a fixed base body.
    const dtDynIndex base = spec.AddBody("base", 0);
    spec.Bodies()[base].inertial.mass = 2.0;
    spec.Bodies()[base].inertial.inertia(0) = 0.05;
    spec.Bodies()[base].inertial.inertia(1) = 0.05;
    spec.Bodies()[base].inertial.inertia(2) = 0.05;

    const dtDynIndex link1 = spec.AddBody("link1", base);
    spec.Bodies()[link1].pos(0) = 0.30;
    spec.Bodies()[link1].inertial.mass = 1.0;
    spec.Bodies()[link1].inertial.pos(0) = 0.15;
    spec.Bodies()[link1].inertial.inertia(0) = 0.01;
    spec.Bodies()[link1].inertial.inertia(1) = 0.02;
    spec.Bodies()[link1].inertial.inertia(2) = 0.02;
    const dtDynIndex j1 = spec.AddJoint(link1, "shoulder", dtDynJointType::Hinge);
    spec.Joints()[j1].axis(2) = 1.0;

    const dtDynIndex link2 = spec.AddBody("link2", link1);
    spec.Bodies()[link2].pos(0) = 0.25;
    spec.Bodies()[link2].inertial.mass = 0.8;
    spec.Bodies()[link2].inertial.pos(0) = 0.125;
    spec.Bodies()[link2].inertial.inertia(0) = 0.008;
    spec.Bodies()[link2].inertial.inertia(1) = 0.012;
    spec.Bodies()[link2].inertial.inertia(2) = 0.012;
    const dtDynIndex j2 = spec.AddJoint(link2, "elbow", dtDynJointType::Hinge);
    spec.Joints()[j2].axis(2) = 1.0;

    const dtDynIndex tool = spec.AddSite(link2, "tool");
    spec.Sites()[tool].pos(0) = 0.20;

    return spec;
}

} // namespace

int main()
{
    const dtDynSpec spec = MakeTwoLinkSpec();
    const dtDynModel model = CompileSpec(spec);

    dtDynData data;
    data.Resize(model); // setup-time allocation; keep this outside hard realtime
    if (!data.IsCompatible(model))
    {
        throw std::runtime_error("dtDynData was not resized for this model");
    }

    // Cache lookup results during setup rather than by name inside the loop.
    const dtDynIndex toolSite = model.SiteId("tool");
    if (toolSite == DTDYN_INVALID_INDEX)
    {
        throw std::runtime_error("missing tool site");
    }

    dt::Math::Vector3<dtDynReal> gravity;
    gravity(0) = 0.0;
    gravity(1) = 0.0;
    gravity(2) = -9.81;

    // One control tick: stage state in preallocated vectors.
    data.qpos(0) = 0.20;
    data.qpos(1) = -0.30;
    data.qvel(0) = 0.10;
    data.qvel(1) = 0.05;

    // Request only the CCRBA outputs consumed by the controller.
    const dtDynCcrbaTermFlags terms =
        DTDYN_CCRBA_TERM_COM_POS |
        DTDYN_CCRBA_TERM_COM_JMAT |
        DTDYN_CCRBA_TERM_CENT_MOM;
    ComputeDynamicsCCRBA(model, data, gravity, terms);

    // Compute one task-space Jacobian for the selected site.
    dtDynDenseMatrix jacp;
    dtDynDenseMatrix jacr;
    dtDynDenseMatrix jacpDot;
    dtDynDenseMatrix jacrDot;
    dt::Math::Vector3<dtDynReal> linearJdotV;
    dt::Math::Vector3<dtDynReal> angularJdotV;
    ComputeSiteJacobianTerms(model,
                             data,
                             toolSite,
                             &jacp,
                             &jacr,
                             &jacpDot,
                             &jacrDot,
                             &linearJdotV,
                             &angularJdotV);

    // Controller-facing ingredients are now in data.Mmat, data.gravityTorq,
    // data.coriolisTorq, data.nonlinTorq, requested CoM/centroidal fields,
    // and the local Jacobian outputs above.
    return 0;
}
```

## What happens in setup

The setup phase establishes ownership and dimensions:

```text
dtDynSpec -> CompileSpec() -> dtDynModel -> dtDynData::Resize(model)
```

`CompileSpec()` assigns deterministic body, joint, site, root, `qpos`, and `qvel` addresses. `dtDynData::Resize(model)` then sizes all state, output, and scratch buffers for those dimensions. Both steps may allocate and should be kept outside the hard realtime loop.

Cache task IDs such as `model.SiteId("tool")` during setup. Name lookup is useful for configuration and diagnostics, but a realtime controller should carry numeric IDs or prebuilt ID arrays into the loop.

## What happens in the loop

Each tick should follow a narrow flow:

1. Write `data.qpos` and `data.qvel` from the robot state estimator.
2. Call `ComputeDynamicsRNEA()` or `ComputeDynamicsCCRBA()` with the smallest useful term mask.
3. Compute only the body, site, or point Jacobian terms required by active tasks.
4. Hand the resulting ingredients to the controller or WBC solver.

The controller can then assemble equations such as

\f[
M(q)\dot{v} + h(q,v) = \tau + J(q)^\mathsf{T} f,
\f]

or task acceleration rows of the form

\f[
a_{\mathrm{task}} = J(q)\dot{v} + \dot{J}(q,v)v.
\f]

dtDynamics stops at the ingredient boundary: it does not solve contact, choose torques, integrate the next state, or enforce actuator limits.

## Choosing the dynamics path

Use `ComputeDynamicsRNEA()` when the loop only needs base bias/gravity/Coriolis-style ingredients. Use `ComputeDynamicsCCRBA()` when the loop also consumes composite or WBC quantities such as CoM position, CoM Jacobian, centroidal momentum matrix, centroidal momentum, or mass-matrix derivative terms.

The term mask is part of the API contract. For example, request `DTDYN_CCRBA_TERM_COM_JMAT` only when `data.comJmat` is consumed that tick, and request `DTDYN_CCRBA_TERM_CMAMAT_DOT` only when the controller uses a centroidal derivative term. Avoid `DTDYN_CCRBA_TERM_ALL` in a hard realtime loop unless every optional output is actually needed.

## Common integration mistakes

- Calling `data.Resize(model)` after entering the control loop.
- Performing name lookup for every task on every tick instead of caching IDs.
- Forgetting that floating-base models can have \f$n_q \neq n_v\f$.
- Treating MuJoCo or Pinocchio as runtime dependencies instead of validation references.
- Expecting dtDynamics to provide simulation stepping, contact resolution, or closed-chain constraint solving.
