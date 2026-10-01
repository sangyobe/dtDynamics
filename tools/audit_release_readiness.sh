#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DTDYNAMICS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ARTF_PREFIX="${ARTF_INSTALL_DIR:-${HOME}/.local/ART_Framework}"
DTMATH_INCLUDE_DIR="${DTDYNAMICS_DTMATH_INCLUDE_DIR:-${ARTF_PREFIX}/include}"
WORK_ROOT="${DTDYNAMICS_AUDIT_ROOT:-$(mktemp -d "${TMPDIR:-/tmp}/dtdynamics-release-audit-XXXXXX")}"
RUN_PACKAGE_VALIDATION=ON
RUN_DOCS_VALIDATION=ON
KEEP_WORK_ROOT=OFF

usage()
{
    cat <<USAGE
Usage: tools/audit_release_readiness.sh [options]

Options:
  --skip-package-validation  Skip package build/test/install validation.
  --skip-docs-validation     Skip Markdown documentation validation.
  --keep-work-root           Keep the temporary audit build root.
  -h, --help                 Show this help text.

Environment:
  ARTF_INSTALL_DIR              Defaults to \$HOME/.local/ART_Framework.
  DTDYNAMICS_DTMATH_INCLUDE_DIR Include root containing dtMath/dtMath.h.
  DTDYNAMICS_AUDIT_ROOT         Reuse an explicit audit work root.

The audit verifies package-level release invariants. It is intentionally
stricter than a plain build: production dtDyn files must remain dtMath-only,
legacy facade/API names must stay out, and the release preset must disable
MuJoCo/Pinocchio reference targets.
USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-package-validation)
            RUN_PACKAGE_VALIDATION=OFF
            ;;
        --skip-docs-validation)
            RUN_DOCS_VALIDATION=OFF
            ;;
        --keep-work-root)
            KEEP_WORK_ROOT=ON
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
    shift
done

cleanup()
{
    if [[ "${KEEP_WORK_ROOT}" == "OFF" && -z "${DTDYNAMICS_AUDIT_ROOT:-}" ]]; then
        rm -rf "${WORK_ROOT}"
    else
        echo "Kept audit root: ${WORK_ROOT}"
    fi
}
trap cleanup EXIT

require_file()
{
    local path="$1"
    if [[ ! -f "${DTDYNAMICS_ROOT}/${path}" ]]; then
        echo "Required file is missing: ${path}" >&2
        exit 1
    fi
}

require_dir()
{
    local path="$1"
    if [[ ! -d "${DTDYNAMICS_ROOT}/${path}" ]]; then
        echo "Required directory is missing: ${path}" >&2
        exit 1
    fi
}

scan_must_be_empty()
{
    local label="$1"
    local pattern="$2"
    local output="${WORK_ROOT}/${label}.txt"
    shift 2

    if grep -R -n -E "${pattern}" "$@" >"${output}"; then
        echo "Release audit failed: ${label}" >&2
        cat "${output}" >&2
        exit 1
    fi
}

cache_bool_must_be_off()
{
    local key="$1"
    local cache_file="$2"
    if ! grep -q "^${key}:.*=OFF$" "${cache_file}"; then
        echo "Release preset did not set ${key}=OFF" >&2
        grep -n "^${key}" "${cache_file}" >&2 || true
        exit 1
    fi
}

mkdir -p "${WORK_ROOT}"

echo "dtDynamics source: ${DTDYNAMICS_ROOT}"
echo "Audit root: ${WORK_ROOT}"
echo "dtMath include: ${DTMATH_INCLUDE_DIR}"
echo "Run package validation: ${RUN_PACKAGE_VALIDATION}"
echo "Run docs validation: ${RUN_DOCS_VALIDATION}"

if [[ ! -f "${DTMATH_INCLUDE_DIR}/dtMath/dtMath.h" ]]; then
    echo "dtMath headers not found at ${DTMATH_INCLUDE_DIR}" >&2
    exit 1
fi

require_file CMakeLists.txt
require_file CMakePresets.json
require_file LICENSE
require_file README.md
require_file tools/package_release.sh
require_file include/dtDynamics/dtArray.h
require_file include/dtDynamics/dtDynTypes.h
require_file include/dtDynamics/dtDynSpec.h
require_file include/dtDynamics/dtDynModel.h
require_file include/dtDynamics/dtDynData.h
require_file include/dtDynamics/dtDynCompile.h
require_file include/dtDynamics/dtDynMath.h
require_file include/dtDynamics/dtDynUtils.h
require_file include/dtDynamics/dtDynKinematics.h
require_file include/dtDynamics/dtDynDynamics.h
require_file docs/manual.md
require_file docs/benchmark-results.md
require_file tools/validate_docs.py
require_file tools/validate_documentation_content.py
require_dir docs/manual
require_dir test/test_dtDynTypes
require_dir test/test_dtDynamicsMujocoRef
require_dir test/test_dtDynamicsPinocchioRef
require_dir benchmark/benchmark_dtDynamics
require_dir benchmark/benchmark_dtDynamicsStages
require_dir benchmark/benchmark_dtDynamicsReferences

scan_must_be_empty \
    production_reference_dependencies \
    '#[[:space:]]*include[[:space:]]*[<"](mujoco|pinocchio)|mj(Model|Data|tNum|tByte|tWarningStat|tDisableBit|tEnableBit)|mj_|pinocchio::' \
    "${DTDYNAMICS_ROOT}/include" "${DTDYNAMICS_ROOT}/src"

scan_must_be_empty \
    legacy_public_api_names \
    'RobotEngineBase|dtDynamicsLegacy|dtDynamics::|DTDYN_COORDINATE_TYPE|\bComputeDynamics[[:space:]]*\(|\bECCRBA\b|\bICCRBA\b' \
    "${DTDYNAMICS_ROOT}/include" "${DTDYNAMICS_ROOT}/src" "${DTDYNAMICS_ROOT}/CMakeLists.txt" "${DTDYNAMICS_ROOT}/cmake"

scan_must_be_empty \
    production_std_vector_usage \
    '#[[:space:]]*include[[:space:]]*<vector>|std::vector[[:space:]]*(<|::)' \
    "${DTDYNAMICS_ROOT}/include" "${DTDYNAMICS_ROOT}/src"

cmake --preset release -S "${DTDYNAMICS_ROOT}" -B "${WORK_ROOT}/release-preset" \
    -DDTDYNAMICS_DTMATH_INCLUDE_DIR="${DTMATH_INCLUDE_DIR}" >/dev/null

cache_file="${WORK_ROOT}/release-preset/CMakeCache.txt"
cache_bool_must_be_off DTDYNAMICS_BUILD_TEST "${cache_file}"
cache_bool_must_be_off DTDYNAMICS_BUILD_BENCHMARK "${cache_file}"
cache_bool_must_be_off DTDYNAMICS_BUILD_REFERENCE_BENCHMARK "${cache_file}"
cache_bool_must_be_off BUILD_DTDYNAMICS_MUJOCO_TEST "${cache_file}"
cache_bool_must_be_off BUILD_DTDYNAMICS_PINOCCHIO_TEST "${cache_file}"

if [[ "${RUN_DOCS_VALIDATION}" == "ON" ]]; then
    python3 "${SCRIPT_DIR}/validate_docs.py"
    python3 "${SCRIPT_DIR}/validate_documentation_content.py"
fi

if [[ "${RUN_PACKAGE_VALIDATION}" == "ON" ]]; then
    DTDYNAMICS_VALIDATE_ROOT="${WORK_ROOT}/package-validation" \
    DTDYNAMICS_DTMATH_INCLUDE_DIR="${DTMATH_INCLUDE_DIR}" \
        "${SCRIPT_DIR}/validate_package.sh"
fi

echo "PASS: dtDynamics release readiness audit completed."
