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
CONSUMER_XML_PATH="${DTDYNAMICS_INSTALL_VALIDATE_XML:-}"
CONSUMER_PREINCLUDE_CONFIG="${DTDYNAMICS_INSTALL_VALIDATE_PREINCLUDE_CONFIG:-}"

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

mkdir -p "${CONSUMER_DIR}"
cat > "${CONSUMER_DIR}/CMakeLists.txt" <<'CONSUMER_CMAKE'
cmake_minimum_required(VERSION 3.16)
project(dtDynamicsInstallConsumer LANGUAGES CXX)

find_package(dtDynamics CONFIG REQUIRED)

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

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{
std::string ReadTextFile(const std::string &path)
{
    std::ifstream input(path);
    if (!input)
    {
        throw std::runtime_error("could not open XML: " + path);
    }

    std::ostringstream stream;
    stream << input.rdbuf();
    return stream.str();
}

std::string ModelXml()
{
#ifdef DTDYNAMICS_INSTALL_VALIDATE_XML_PATH
    return ReadTextFile(DTDYNAMICS_INSTALL_VALIDATE_XML_PATH);
#else
    return "<mujoco>"
           "<worldbody>"
           "<body name=\"base\">"
           "<freejoint name=\"root\"/>"
           "<body name=\"link\" pos=\"0 0 0.2\">"
           "<inertial pos=\"0 0 0\" mass=\"1\" diaginertia=\"0.01 0.01 0.01\"/>"
           "<joint name=\"hinge\" type=\"hinge\" axis=\"0 0 1\"/>"
           "</body>"
           "</body>"
           "</worldbody>"
           "</mujoco>";
#endif
}
} // namespace

int main()
{
    const auto spec = dt::Dynamics::dtDynSpec::DecodeXml(ModelXml());

    dt::Dynamics::dtDynModel model;
    model = dt::Dynamics::CompileSpec(spec);

    dt::Dynamics::dtDynData data;
    data.Resize(model);
    return 0;
}
CONSUMER_CPP

cmake -S "${CONSUMER_DIR}" -B "${CONSUMER_BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_PREFIX_PATH="${INSTALL_PREFIX}" \
    -DDTDYNAMICS_DTMATH_INCLUDE_DIR="${DTMATH_INCLUDE_DIR}"
if [[ -n "${CONSUMER_XML_PATH}" ]]; then
    cmake -S "${CONSUMER_DIR}" -B "${CONSUMER_BUILD_DIR}" \
        -DDTDYNAMICS_INSTALL_VALIDATE_XML_PATH="${CONSUMER_XML_PATH}"
fi
if [[ -n "${CONSUMER_PREINCLUDE_CONFIG}" ]]; then
    cmake -S "${CONSUMER_DIR}" -B "${CONSUMER_BUILD_DIR}" \
        -DDTDYNAMICS_INSTALL_VALIDATE_PREINCLUDE_CONFIG="${CONSUMER_PREINCLUDE_CONFIG}"
fi
cmake --build "${CONSUMER_BUILD_DIR}" -- -j2
"${CONSUMER_BUILD_DIR}/dtDynamicsInstallConsumer"

echo "PASS: dtDynamics install/export validation succeeded."
echo "Install prefix: ${INSTALL_PREFIX}"
