#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DTDYNAMICS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
ARTF_PREFIX="${ARTF_INSTALL_DIR:-${HOME}/.local/ART_Framework}"
DTMATH_INCLUDE_DIR="${DTDYNAMICS_DTMATH_INCLUDE_DIR:-${ARTF_PREFIX}/include}"
VALIDATION_DEPS_DIR="${DTDYNAMICS_VALIDATION_DEPS_DIR:-${DTDYNAMICS_ROOT}/.validation_deps}"
MUJOCO_VERSION="${DTDYNAMICS_MUJOCO_VERSION:-3.8.0}"
PINOCCHIO_VERSION="${DTDYNAMICS_PINOCCHIO_VERSION:-3.9.0}"
MUJOCO_ROOT="${DTDYNAMICS_MUJOCO_ROOT:-${VALIDATION_DEPS_DIR}/mujoco-${MUJOCO_VERSION}}"
PINOCCHIO_ROOT="${DTDYNAMICS_PINOCCHIO_ROOT:-${VALIDATION_DEPS_DIR}/pinocchio-${PINOCCHIO_VERSION}}"
WORK_ROOT="${DTDYNAMICS_VALIDATE_ROOT:-$(mktemp -d "${TMPDIR:-/tmp}/dtdynamics-package-validate-XXXXXX")}"
BUILD_DIR="${WORK_ROOT}/build"
INSTALL_VALIDATE_ROOT="${WORK_ROOT}/install-export"
BUILD_BENCHMARKS=ON
REFERENCE_TESTS=ON
KEEP_WORK_ROOT=OFF

usage()
{
    cat <<USAGE
Usage: tools/validate_package.sh [options]

Options:
  --skip-reference-tests  Build only native tests; do not require MuJoCo/Pinocchio.
  --skip-benchmarks       Do not build benchmark executables.
  --keep-work-root        Keep the temporary validation build root.
  -h, --help              Show this help text.

Environment:
  CMAKE_BUILD_TYPE                  Defaults to Release.
  ARTF_INSTALL_DIR                  Defaults to \$HOME/.local/ART_Framework.
  DTDYNAMICS_DTMATH_INCLUDE_DIR     Include root containing dtMath/dtMath.h.
  DTDYNAMICS_VALIDATION_DEPS_DIR    Defaults to <package>/.validation_deps.
  DTDYNAMICS_VALIDATE_ROOT          Reuse an explicit build root.

The validation build uses the checked-in CMakePresets.json as its baseline:
  validation         Full native plus MuJoCo/Pinocchio reference validation.
  native-validation Native validation without MuJoCo/Pinocchio.
USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-reference-tests)
            REFERENCE_TESTS=OFF
            ;;
        --skip-benchmarks)
            BUILD_BENCHMARKS=OFF
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

if [[ ! -f "${DTMATH_INCLUDE_DIR}/dtMath/dtMath.h" ]]; then
    echo "dtMath headers not found at ${DTMATH_INCLUDE_DIR}" >&2
    exit 1
fi
mkdir -p "${WORK_ROOT}"

if [[ "${REFERENCE_TESTS}" == "ON" ]]; then
    if [[ ! -f "${MUJOCO_ROOT}/include/mujoco/mujoco.h" ]]; then
        echo "MuJoCo ${MUJOCO_VERSION} validation dependency not found at ${MUJOCO_ROOT}" >&2
        echo "Run tools/install_validation_deps.sh --mujoco or pass --skip-reference-tests." >&2
        exit 1
    fi
    if [[ ! -f "${PINOCCHIO_ROOT}/lib/cmake/pinocchio/pinocchioConfig.cmake" ]]; then
        echo "Pinocchio ${PINOCCHIO_VERSION} validation dependency not found at ${PINOCCHIO_ROOT}" >&2
        echo "Run tools/install_validation_deps.sh --pinocchio or pass --skip-reference-tests." >&2
        exit 1
    fi
fi

echo "dtDynamics source: ${DTDYNAMICS_ROOT}"
echo "Validation root: ${WORK_ROOT}"
echo "Build type: ${BUILD_TYPE}"
echo "dtMath include: ${DTMATH_INCLUDE_DIR}"
echo "Validation deps: ${VALIDATION_DEPS_DIR}"
echo "Reference tests: ${REFERENCE_TESTS}"
echo "Build benchmarks: ${BUILD_BENCHMARKS}"

if [[ "${REFERENCE_TESTS}" == "ON" ]]; then
    CMAKE_PRESET=validation
    REFERENCE_BENCHMARKS=ON
else
    CMAKE_PRESET=native-validation
    REFERENCE_BENCHMARKS=OFF
fi
echo "CMake preset: ${CMAKE_PRESET}"

SCAN_OUTPUT="${WORK_ROOT}/production-source-scan.txt"
if grep -R -n -E '#[[:space:]]*include[[:space:]]*[<"](mujoco|pinocchio)|mj(Model|Data|tNum|tByte|tWarningStat|tDisableBit|tEnableBit)|mj_|pinocchio::' \
    "${DTDYNAMICS_ROOT}/include" "${DTDYNAMICS_ROOT}/src" >"${SCAN_OUTPUT}"; then
    echo "Production include/src contains forbidden MuJoCo or Pinocchio API references:" >&2
    cat "${SCAN_OUTPUT}" >&2
    exit 1
fi

cmake --preset "${CMAKE_PRESET}" -S "${DTDYNAMICS_ROOT}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DDTDYNAMICS_BUILD_BENCHMARK="${BUILD_BENCHMARKS}" \
    -DDTDYNAMICS_BUILD_REFERENCE_BENCHMARK="${REFERENCE_BENCHMARKS}" \
    -DDTDYNAMICS_VALIDATION_DEPS_DIR="${VALIDATION_DEPS_DIR}" \
    -DDTDYNAMICS_DTMATH_INCLUDE_DIR="${DTMATH_INCLUDE_DIR}"
cmake --build "${BUILD_DIR}" -- -j2
ctest --test-dir "${BUILD_DIR}" --output-on-failure

DTDYNAMICS_INSTALL_VALIDATE_ROOT="${INSTALL_VALIDATE_ROOT}" \
CMAKE_BUILD_TYPE="${BUILD_TYPE}" \
DTDYNAMICS_DTMATH_INCLUDE_DIR="${DTMATH_INCLUDE_DIR}" \
    "${SCRIPT_DIR}/validate_install_export.sh"

if [[ "${KEEP_WORK_ROOT}" == "OFF" && -z "${DTDYNAMICS_VALIDATE_ROOT:-}" ]]; then
    rm -rf "${WORK_ROOT}"
else
    echo "Kept validation root: ${WORK_ROOT}"
fi

echo "PASS: dtDynamics package validation completed."
