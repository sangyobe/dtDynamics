#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DTDYNAMICS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
ARTF_PREFIX="${ARTF_INSTALL_DIR:-${HOME}/.local/ART_Framework}"
DTMATH_INCLUDE_DIR="${DTDYNAMICS_DTMATH_INCLUDE_DIR:-${ARTF_PREFIX}/include}"
WORK_ROOT="${DTDYNAMICS_INSTALL_VALIDATE_ROOT:-$(mktemp -d "${TMPDIR:-/tmp}/dtdynamics-install-export-XXXXXX")}"
BUILD_DIR="${WORK_ROOT}/build"
INSTALL_PREFIX="${WORK_ROOT}/install"
CONSUMER_DIR="${WORK_ROOT}/consumer"
CONSUMER_BUILD_DIR="${WORK_ROOT}/consumer-build"
CONSUMER_FILE_BUILD_DIR="${WORK_ROOT}/consumer-file-build"
CONSUMER_XML_PATH="${DTDYNAMICS_INSTALL_VALIDATE_XML:-${CONSUMER_DIR}/install-consumer-model.xml}"
CONSUMER_PREINCLUDE_CONFIG="${DTDYNAMICS_INSTALL_VALIDATE_PREINCLUDE_CONFIG:-}"
EXPECTED_VERSION="${DTDYNAMICS_EXPECTED_VERSION:-0.1.1}"

if [[ ! -f "${DTMATH_INCLUDE_DIR}/dtMath/dtMath.h" ]]; then
    echo "dtMath headers not found at ${DTMATH_INCLUDE_DIR}" >&2
    echo "Set DTDYNAMICS_DTMATH_INCLUDE_DIR to the include root containing dtMath/dtMath.h." >&2
    exit 1
fi

cmake --preset release -S "${DTDYNAMICS_ROOT}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DDTDYNAMICS_DTMATH_INCLUDE_DIR="${DTMATH_INCLUDE_DIR}"
cmake --build "${BUILD_DIR}" --target install -- -j2

CONFIG_DIR="${INSTALL_PREFIX}/lib/cmake/dtDynamics"
if [[ ! -f "${CONFIG_DIR}/dtDynamicsTargets.cmake" ]]; then
    echo "Installed dtDynamicsTargets.cmake was not found under ${CONFIG_DIR}" >&2
    exit 1
fi

if ! grep -R "add_library(dt::Dynamics" "${CONFIG_DIR}" >/dev/null; then
    echo "Installed package does not export target dt::Dynamics." >&2
    exit 1
fi

if grep -R "dtDynamics::" "${CONFIG_DIR}" >/dev/null; then
    echo "Installed package still exports a legacy dtDynamics:: target." >&2
    exit 1
fi

if grep -R -i -E "mujoco|pinocchio" "${CONFIG_DIR}" >/dev/null; then
    echo "Installed production package config contains MuJoCo or Pinocchio references." >&2
    exit 1
fi

DOC_DIR="${INSTALL_PREFIX}/share/doc/dtDynamics"
if [[ ! -f "${DOC_DIR}/manual.md" ]]; then
    echo "Installed package manual was not found under ${DOC_DIR}" >&2
    exit 1
fi
if [[ ! -f "${DOC_DIR}/benchmark-results.md" ]]; then
    echo "Installed benchmark results note was not found under ${DOC_DIR}" >&2
    exit 1
fi

python3 - "${INSTALL_PREFIX}/include/dtDynamics" "${EXPECTED_VERSION}" <<'PY'
import pathlib
import re
import sys

include_dir = pathlib.Path(sys.argv[1])
expected = sys.argv[2]
mismatches = []
versioned_headers = 0
for header in sorted(include_dir.glob("*.h")):
    match = re.search(r"\\version\s+(\S+)", header.read_text())
    if match is None:
        continue
    versioned_headers += 1
    if match.group(1) != expected:
        mismatches.append(f"{header.name}: {match.group(1)}")
if versioned_headers == 0:
    raise SystemExit("Installed public headers expose no version declarations")
if mismatches:
    raise SystemExit(
        f"Installed public-header versions do not equal {expected}: " + ", ".join(mismatches)
    )
PY

python3 - "${DOC_DIR}" "${EXPECTED_VERSION}" <<'PY'
import pathlib
import re
import sys

doc_dir = pathlib.Path(sys.argv[1])
expected = sys.argv[2]
claims = []
for svg in sorted(doc_dir.rglob("*.svg")):
    for match in re.finditer(r"package version\s+([0-9]+\.[0-9]+\.[0-9]+)", svg.read_text(), re.IGNORECASE):
        claims.append((svg.relative_to(doc_dir), match.group(1)))
if not claims:
    raise SystemExit("Installed current SVG documentation exposes no package-version claim")
mismatches = [f"{path}: {version}" for path, version in claims if version != expected]
if mismatches:
    raise SystemExit(
        f"Installed current SVG package-version claims do not equal {expected}: " + ", ".join(mismatches)
    )
PY

mkdir -p "${CONSUMER_DIR}"
cat > "${CONSUMER_DIR}/CMakeLists.txt" <<'CONSUMER_CMAKE'
cmake_minimum_required(VERSION 3.16)
project(dtDynamicsInstallConsumer LANGUAGES CXX)

if(NOT DEFINED DTDYNAMICS_EXPECTED_VERSION)
    message(FATAL_ERROR "DTDYNAMICS_EXPECTED_VERSION must be provided by the validation driver")
endif()
find_package(dtDynamics ${DTDYNAMICS_EXPECTED_VERSION} EXACT CONFIG REQUIRED)
if(NOT dtDynamics_VERSION STREQUAL DTDYNAMICS_EXPECTED_VERSION)
    message(FATAL_ERROR
        "Installed dtDynamics version ${dtDynamics_VERSION} does not equal ${DTDYNAMICS_EXPECTED_VERSION}")
endif()

add_executable(dtDynamicsInstallConsumer main.cpp)
target_link_libraries(dtDynamicsInstallConsumer PRIVATE dt::Dynamics)
if(DEFINED DTDYNAMICS_INSTALL_VALIDATE_XML_PATH)
    target_compile_definitions(dtDynamicsInstallConsumer PRIVATE
        DTDYNAMICS_INSTALL_VALIDATE_XML_PATH="${DTDYNAMICS_INSTALL_VALIDATE_XML_PATH}")
endif()
if(DEFINED DTDYNAMICS_INSTALL_VALIDATE_PREINCLUDE_CONFIG)
    target_compile_definitions(dtDynamicsInstallConsumer PRIVATE
        DTDYNAMICS_INSTALL_VALIDATE_PREINCLUDE_CONFIG="${DTDYNAMICS_INSTALL_VALIDATE_PREINCLUDE_CONFIG}")
endif()
CONSUMER_CMAKE

cat > "${CONSUMER_DIR}/main.cpp" <<'CONSUMER_CPP'
#ifdef DTDYNAMICS_INSTALL_VALIDATE_PREINCLUDE_CONFIG
#include DTDYNAMICS_INSTALL_VALIDATE_PREINCLUDE_CONFIG
#endif

#include <dtDynamics/dtDynCompile.h>
#include <dtDynamics/dtDynData.h>
#include <dtDynamics/dtDynDynamics.h>
#include <dtDynamics/dtDynKinematics.h>
#include <dtDynamics/dtDynModel.h>
#include <dtDynamics/dtDynSpec.h>
#include <dtDynamics/dtDynValidation.h>

#include <stdexcept>
#include <string>

namespace
{
dt::Dynamics::dtDynSpec LoadConsumerSpec()
{
#ifdef DTDYNAMICS_INSTALL_VALIDATE_XML_PATH
    return dt::Dynamics::dtDynSpec::LoadXmlFile(
        DTDYNAMICS_INSTALL_VALIDATE_XML_PATH);
#else
    return dt::Dynamics::dtDynSpec::DecodeXml(
        "<mujoco>"
        "<worldbody>"
        "<body name=\"base\">"
        "<freejoint name=\"root\"/>"
        "<body name=\"link\" pos=\"0 0 0.2\">"
        "<inertial pos=\"0 0 0\" mass=\"1\" diaginertia=\"0.01 0.01 0.01\"/>"
        "<joint name=\"hinge\" type=\"hinge\" axis=\"0 0 1\"/>"
        "</body>"
        "</body>"
        "</worldbody>"
        "</mujoco>");
#endif
}
} // namespace

int main()
{
    const auto spec = LoadConsumerSpec();
    if (!dt::Dynamics::ValidateSpec(spec))
    {
        throw std::runtime_error("installed ValidateSpec rejected consumer model");
    }
    const auto model = dt::Dynamics::CompileSpecChecked(spec);

    dt::Dynamics::dtDynData data;
    data.Resize(model);

    const auto missing = "__dtdynamics_install_consumer_missing__";
    if (spec.BodyId(missing) != dt::Dynamics::DTDYN_INVALID_INDEX ||
        spec.JointId(missing) != dt::Dynamics::DTDYN_INVALID_INDEX ||
        spec.SiteId(missing) != dt::Dynamics::DTDYN_INVALID_INDEX ||
        spec.ActuatorId(missing) != dt::Dynamics::DTDYN_INVALID_INDEX ||
        model.BodyId(missing) != dt::Dynamics::DTDYN_INVALID_INDEX ||
        model.JointId(missing) != dt::Dynamics::DTDYN_INVALID_INDEX ||
        model.SiteId(missing) != dt::Dynamics::DTDYN_INVALID_INDEX ||
        model.ActuatorId(missing) != dt::Dynamics::DTDYN_INVALID_INDEX)
    {
        throw std::runtime_error("installed lookup sentinel contract failed");
    }

    dt::Dynamics::dtDynSpec zeroSpec;
    if (!dt::Dynamics::ValidateSpec(zeroSpec))
    {
        throw std::runtime_error("installed zero-DoF validation failed");
    }
    const auto zeroModel = dt::Dynamics::CompileSpecChecked(zeroSpec);
    dt::Dynamics::dtDynData zeroData;
    zeroData.Resize(zeroModel);
    if (zeroModel.nq != 0 || zeroModel.nv != 0 ||
        !zeroData.IsCompatible(zeroModel))
    {
        throw std::runtime_error("installed zero-DoF resize failed");
    }
    return 0;
}
CONSUMER_CPP

if [[ -z "${DTDYNAMICS_INSTALL_VALIDATE_XML:-}" ]]; then
    cat > "${CONSUMER_XML_PATH}" <<'CONSUMER_XML'
<mujoco>
  <worldbody>
    <body name="base">
      <freejoint name="root"/>
      <body name="link" pos="0 0 0.2">
        <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
        <joint name="hinge" type="hinge" axis="0 0 1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
CONSUMER_XML
fi

# First compile and run the installed in-memory DecodeXml path.
cmake -S "${CONSUMER_DIR}" -B "${CONSUMER_BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_PREFIX_PATH="${INSTALL_PREFIX}" \
    -DDTDYNAMICS_DTMATH_INCLUDE_DIR="${DTMATH_INCLUDE_DIR}" \
    -DDTDYNAMICS_EXPECTED_VERSION="${EXPECTED_VERSION}"
if [[ -n "${CONSUMER_PREINCLUDE_CONFIG}" ]]; then
    cmake -S "${CONSUMER_DIR}" -B "${CONSUMER_BUILD_DIR}" \
        -DDTDYNAMICS_INSTALL_VALIDATE_PREINCLUDE_CONFIG="${CONSUMER_PREINCLUDE_CONFIG}"
fi
cmake --build "${CONSUMER_BUILD_DIR}" -- -j2
"${CONSUMER_BUILD_DIR}/dtDynamicsInstallConsumer"
echo "PASS: installed DecodeXml consumer executed."

# Then independently compile and run the installed package-owned LoadXmlFile path.
cmake -S "${CONSUMER_DIR}" -B "${CONSUMER_FILE_BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_PREFIX_PATH="${INSTALL_PREFIX}" \
    -DDTDYNAMICS_DTMATH_INCLUDE_DIR="${DTMATH_INCLUDE_DIR}" \
    -DDTDYNAMICS_EXPECTED_VERSION="${EXPECTED_VERSION}" \
    -DDTDYNAMICS_INSTALL_VALIDATE_XML_PATH="${CONSUMER_XML_PATH}"
if [[ -n "${CONSUMER_PREINCLUDE_CONFIG}" ]]; then
    cmake -S "${CONSUMER_DIR}" -B "${CONSUMER_FILE_BUILD_DIR}" \
        -DDTDYNAMICS_INSTALL_VALIDATE_PREINCLUDE_CONFIG="${CONSUMER_PREINCLUDE_CONFIG}"
fi
cmake --build "${CONSUMER_FILE_BUILD_DIR}" -- -j2
"${CONSUMER_FILE_BUILD_DIR}/dtDynamicsInstallConsumer"
echo "PASS: installed LoadXmlFile consumer executed: ${CONSUMER_XML_PATH}"

echo "PASS: dtDynamics install/export validation succeeded."
echo "Install prefix: ${INSTALL_PREFIX}"
