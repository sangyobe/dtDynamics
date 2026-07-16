# Overview {#dtdynamics_overview}

dtDynamics is a standalone package for computing rigid-body dynamics ingredients used by realtime controllers and whole-body control (WBC) stacks. The package is designed around explicit model/data separation, predictable indexing, and documented compute paths that can run without measured C++ `operator new` allocation after the user has compiled a model, resized `dtDynData`, and pre-sized any caller-owned outputs.

## What the package is

The package provides:

- `dtDynSpec`: a mutable model specification, typically loaded from a supported XML subset or built programmatically.
- `dtDynModel`: a compiled, immutable model with body, joint, coordinate, and site addresses.
- `dtDynData`: mutable runtime storage for state, kinematics, dynamics, Jacobians, and scratch arrays.
- Kinematics calls for body/site transforms and velocity-dependent quantities.
- RNEA and CCRBA dynamics calls for controller-facing terms.
- Body/site/point Jacobian utilities and derivative products.

## What the package is not

The package does not provide a contact solver, actuator model, simulator integration loop, robot controller, public MuJoCo dependency, or closed-chain solver. MuJoCo and Pinocchio are validation and benchmark references only.

## Manual philosophy

Like the MuJoCo manual, this documentation explains concepts before individual functions. For each major API, read the corresponding convention section first: most bugs in dynamics integration come from coordinate order, frame convention, lifetime/ownership, or assuming that a quantity exists before its prerequisite computation.
