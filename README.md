# dtDynamics

`dtDynamics` is the standalone ART dynamics ingredient package. It provides
dtMath-based rigid-body dynamics quantities for whole-body control, state
estimation, and model-based planning. The package computes ingredients such as
mass matrix, nonlinear torque, gravity torque, Coriolis torque, Jacobians,
Jacobian derivatives, CoM quantities, centroidal momentum matrix, and related
centroidal derivative terms. It does not compute the final controller torque
command.

The production library is independent of MuJoCo and Pinocchio. MuJoCo 3.8.0 and
Pinocchio 3.9.0 are used only by validation tests and benchmarks under the
private `.validation_deps` cache.

## Layout

- `include/dtDynamics/`: public dtDyn API headers in namespace `dt::Dynamics`
- `src/`: production dtMath-only implementation
- `test/`: native, MuJoCo-reference, and Pinocchio-reference validation tests
- `benchmark/`: runtime and numerical comparison benchmarks
- `tools/`: validation dependency and install/export helper scripts
- `docs/`: package manual, benchmark results, figures, and release notes

## Dependencies

Production dependency:

- dtMath installed under `${ARTF_INSTALL_DIR:-~/.local/ART_Framework}/include`

Validation-only dependencies:

- MuJoCo 3.8.0
- Pinocchio 3.9.0

Do not install MuJoCo or Pinocchio into the ART Framework prefix for production
consumers. Use `.validation_deps` or explicit validation root overrides instead.

## Build And Install

For CMake 3.21 or newer, use the checked-in presets:

```bash
cmake --preset validation
cmake --build --preset validation -- -j2
ctest --preset validation

cmake --preset release
cmake --build --preset release -- -j2
cmake --install build/release --prefix "${ARTF_INSTALL_DIR:-$HOME/.local/ART_Framework}"

# Optional: validate against a versioned dtMath include root.
export ARTF_INSTALL_DIR="${ARTF_INSTALL_DIR:-$HOME/.local/ART_Framework}"
cmake --preset release -B build/release-dtmath-3 \
  -DDTDYNAMICS_DTMATH_INCLUDE_DIR="${ARTF_INSTALL_DIR}/include/dtMath_3_0_0"
cmake --build build/release-dtmath-3 -- -j2
cmake --install build/release-dtmath-3 --prefix "${ARTF_INSTALL_DIR}"
```

Downstream projects should consume the installed package target:

```cmake
find_package(dtDynamics CONFIG REQUIRED)
target_link_libraries(my_controller PRIVATE dt::Dynamics)
```

## Validation Dependencies

Populate the private validation cache when MuJoCo/Pinocchio tests or reference
benchmarks are needed:

```bash
tools/install_validation_deps.sh --all
```

The cache lives in `.validation_deps/` by default. It is ignored by Git and is
not part of the production package.

## Package Boundary Check

Run the package validator before handing the package to downstream users:

```bash
tools/validate_package.sh
```

This performs the production source boundary scan, configures a Release build,
builds native/reference tests and benchmark executables, runs `ctest`, and then
executes the install/export validator.

For a narrower installed-package check:

```bash
tools/validate_install_export.sh
```

The narrower check verifies that the installed package exports `dt::Dynamics`,
production config files do not reference MuJoCo or Pinocchio, package docs are
installed, and a minimal downstream consumer can link against the package.

Before a release or downstream handoff, run the stricter audit:

```bash
tools/audit_release_readiness.sh
```

It adds static release-invariant scans on top of the package validator: required
package artifacts, no legacy facade/API names in production files, no MuJoCo or
Pinocchio production API use, no `std::vector` use in native production files,
and a production-only `release` preset.

To create a reproducible source handoff archive from a clean committed tree:

```bash
tools/package_release.sh --version v0.1.0-rc3 --output-dir dist
```

The packaging helper runs the release-readiness audit by default, archives git
`HEAD`, and writes a `.sha256` checksum next to the archive.

## Documentation

Start with `docs/manual.md` for API conventions, computation pipeline, XML
subset, validation workflow, benchmark workflow, limitations, and roadmap. See
`docs/benchmark-results.md` for the current measured baseline. Release-candidate
handoff notes live under `docs/release-notes/`; the current corrected candidate is
`docs/release-notes/v0.1.0-rc3.md`; rc2 and rc1 are preserved as
historical context.

## License

Copyright (c) 2026 RoboticsLab ART. All rights reserved. See [`LICENSE`](LICENSE).
