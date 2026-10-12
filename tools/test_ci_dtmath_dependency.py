#!/usr/bin/env python3
"""Lock CI to the immutable dtMath producer revision used by Polyspace."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
CI = ROOT / ".gitlab-ci.yml"
PREPARE = ROOT / "ci" / "prepare_dtmath_dependency.sh"
DTMATH_MERGE_SHA = "b6cb3af6d0ec930d3bcfc030b146380d0fc56cfd"


def main() -> None:
    ci = CI.read_text(encoding="utf-8")
    assert PREPARE.is_file(), "missing CI dtMath immutable-checkout helper"
    helper = PREPARE.read_text(encoding="utf-8")
    build = (ROOT / "ci" / "build.sh").read_text(encoding="utf-8")

    assert f'DTDYNAMICS_DTMATH_COMMIT: "{DTMATH_MERGE_SHA}"' in ci
    assert 'DTDYNAMICS_DTMATH_INCLUDE_DIR: "${CI_PROJECT_DIR}/.ci/dtmath/include"' in ci
    assert "before_script:" in ci
    assert "- bash ci/prepare_dtmath_dependency.sh" in ci
    assert "python3 tools/test_ci_dtmath_dependency.py" in ci

    for required in (
        'CI_JOB_TOKEN',
        'DTDYNAMICS_DTMATH_COMMIT',
        'DTDYNAMICS_DTMATH_INCLUDE_DIR',
        'GIT_ASKPASS',
        'git clone --no-checkout',
        'fetch --depth=1 origin "${DTDYNAMICS_DTMATH_COMMIT}"',
        'checkout --detach "${DTDYNAMICS_DTMATH_COMMIT}"',
        'rev-parse HEAD',
        'dtMath/dtMath.h',
    ):
        assert required in helper, f"missing immutable-producer control: {required}"

    assert "https://gitlab-ci-token:${CI_JOB_TOKEN}" not in helper
    assert "CI_JOB_TOKEN" not in helper.split("git clone", 1)[1], (
        "CI job token must be supplied only through the ephemeral askpass helper"
    )
    assert 'dtmath_cmake_args+=("-DDTDYNAMICS_DTMATH_INCLUDE_DIR=${DTDYNAMICS_DTMATH_INCLUDE_DIR}")' in build
    assert '"${dtmath_cmake_args[@]}"' in build

    print("CI immutable dtMath dependency contract: PASS")


if __name__ == "__main__":
    main()
