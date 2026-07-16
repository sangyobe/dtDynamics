# Validation {#dtdynamics_validation}

@dotfile diagrams/validation_refs.dot

Validation is external to the production package boundary.

## Native validation

Native consistency checks include matrix symmetry and cross-path residuals. For example, mass-matrix symmetry is evaluated through

\f[
\epsilon_M = \lVert M - M^\mathsf{T} \rVert_{\infty},
\f]

and reference comparisons use residuals of the form

\f[
\epsilon_y = \lVert y_{\mathrm{dtDynamics}} - T(y_{\mathrm{ref}}) \rVert,
\f]

where \f$T(\cdot)\f$ applies any required coordinate, frame, or row-order transform before comparison.

Native tests check package semantics that do not require external rigid-body libraries: dimensions, body/site indexing, coordinate addresses, fixed bodies, multi-root handling, Jacobian shapes, and internal consistency.

## MuJoCo reference validation

MuJoCo is used as a reference for overlapping tree-model quantities and indexing conventions. It is not a production dependency and should not appear in public dtDynamics headers or production link interfaces.

## Pinocchio reference validation

Pinocchio is used for quantities not directly exposed by MuJoCo or for independent dynamics checks. Comparisons must account for frame conventions, free-flyer ordering, centroidal row order, and intentionally skipped non-comparable rows.

## Runtime validation

Controller integration requires a runtime gate in addition to package tests. LeoQuad's headless trot validation checks that the installed package can drive the controller path without display/GLFW and can optionally report dynamics timing splits.


## Current validation snapshot

The documentation/build validation used for this manual is intentionally reproducible from the repository rather than embedded as an unverifiable claim. The current native validation gate is:

```bash
cmake --preset native-validation
cmake --build --preset native-validation -- -j2
ctest --preset native-validation
```

The latest manual-authoring run completed the native-validation preset (`test_dtDynTypes`) with `100% tests passed, 0 tests failed out of 1`. The same run regenerated this Doxygen manual with `tools/validate_docs.sh`, zero Doxygen warnings, no missing generated-image references, and locally generated formula images. Treat this as a checked-working-tree snapshot, not a substitute for rerunning the full external MuJoCo/Pinocchio validation path after model, compiler, or dependency changes.

The public validation boundary is conservative: MuJoCo and Pinocchio rows in @ref md_docs_benchmark_results are references for overlapping quantities and coordinate/frame transforms. They are not runtime dependencies of the production dtDynamics APIs.
