# dtDynamics Manual {#mainpage}

This manual is the source for the generated dtDynamics documentation. It is written as a conceptual manual first and an API index second: users should understand the model/data lifecycle, coordinate conventions, realtime allocation rules, and validation scope before calling the API.

## Build the manual

```bash
cd /path/to/dtDynamics
tools/validate_docs.sh
```

Generated HTML is written under `build/docs/doxygen/html/` and is not meant to be committed by default.

## Manual map

- @ref dtdynamics_overview "Overview": package scope and non-goals.
- @ref dtdynamics_modeling "Modeling and Data Lifecycle": `dtDynSpec`, `dtDynModel`, `dtDynData`, tree topology, indexing, and coordinates.
- @ref dtdynamics_computation "Computation Pipeline": FK, RNEA, CCRBA, Jacobians, CoM, centroidal, and term masks.
- @ref dtdynamics_example "Minimal End-to-End Usage Example": setup-to-loop code flow for a small tree model.
- @ref dtdynamics_realtime "Realtime Resource Model": realtime lifecycle, allocation boundaries, and resource measurements.
- @ref dtdynamics_validation "Validation": native/reference validation and convention transforms.
- @ref dtdynamics_benchmarks "Benchmarks and Resource Reports": benchmark binaries, CSV sections, resource columns, and interpretation.
- @ref dtdynamics_integration "Controller Integration": controller/WBC integration pattern.
- @ref dtdynamics_api_map "API Map": curated public API map.
- @ref dtdynamics_limitations "Limitations and Scope Boundaries": conservative scope limits and future work.
- @ref dtdynamics_release "Release and Package Boundary Checks": release/package-boundary checks.

## Design principle

dtDynamics is a standalone dynamics ingredient engine. It computes control-relevant quantities such as mass matrices, bias/gravity/Coriolis terms, Jacobians, derivative products, CoM quantities, and centroidal quantities. It is not a simulator, contact solver, torque controller, or MuJoCo wrapper.
