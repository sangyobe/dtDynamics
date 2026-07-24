# API Map {#dtdynamics_api_map}

Author: Joonhee Jo

This page is a curated map to the generated C++ API pages.

## Model lifecycle

- `dtDynSpec`: mutable construction/specification layer.
- `CompileSpec`: compiles a spec into a model.
- `dtDynModel`: immutable compiled topology and address layer.
- `dtDynData`: mutable preallocated runtime storage.

## Compute calls

- `ForwardKinematics`: update body/site kinematic quantities.
- `ComputeDynamicsRNEA`: lighter RNEA-based dynamics path.
- `ComputeDynamicsCCRBA`: composite/centroidal WBC ingredient path.
- Body/site/point Jacobian functions: task-space kinematics and derivative products.

Read the conceptual pages before relying on generated signatures alone.
