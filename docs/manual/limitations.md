# Limitations and Scope Boundaries {#dtdynamics_limitations}

- Core support is for tree-structured rigid-body models.
- Fixed-base manipulators, multi-root fixed/floating models, and floating-base quadrupeds are in scope when validated.
- Closed-chain/parallel mechanisms are not a supported solved-dynamics feature unless a specific equality-constraint layer and validation are implemented.
- Contact, collision, actuator dynamics, simulation stepping, and torque-controller policy are out of scope.
- MuJoCo and Pinocchio are validation references only, not production dependencies.
- `dtMath` is an implementation dependency, not the scientific contribution by itself.
