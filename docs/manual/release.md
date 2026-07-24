# Release and Package Boundary Checks {#dtdynamics_release}

Author: Joonhee Jo

Before release or installation claims, run package validation from a clean temporary build root where possible.

```bash
tools/audit_release_readiness.sh
```

The audit should verify:

- production headers/sources do not expose MuJoCo or Pinocchio APIs;
- the package installs directly into the ART Framework prefix, not a nested package prefix;
- installed CMake config exports `dt::Dynamics`;
- a downstream `find_package(dtDynamics CONFIG REQUIRED)` consumer builds;
- native and reference validation tests pass when validation dependencies are present.

Do not mutate historical tags or publish release artifacts without explicit approval.
