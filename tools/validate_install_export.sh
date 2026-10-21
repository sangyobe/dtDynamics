#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DTDYNAMICS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
ARTF_PREFIX="${ARTF_INSTALL_DIR:-${HOME}/.local/ART_Framework}"
DTMATH_INCLUDE_DIR="${DTDYNAMICS_DTMATH_INCLUDE_DIR:-${ARTF_PREFIX}/include}"
WORK_ROOT="${DTDYNAMICS_INSTALL_VALIDATE_ROOT:-$(mktemp -d "${TMPDIR:-/tmp}/dtdynamics-install-export-XXXXXX")}"
INSTALL_LIBDIR="${DTDYNAMICS_INSTALL_LIBDIR:-lib}"
INSTALL_DOCDIR="${DTDYNAMICS_INSTALL_DOCDIR:-share/doc/dtDynamics}"
BUILD_DIR="${WORK_ROOT}/build"
INSTALL_PREFIX="${WORK_ROOT}/install"
RELOCATED_PREFIX="${WORK_ROOT}/relocated-install"
CONSUMER_DIR="${WORK_ROOT}/consumer"
CONSUMER_BUILD_DIR="${WORK_ROOT}/consumer-build"
CONSUMER_FILE_BUILD_DIR="${WORK_ROOT}/consumer-file-build"
CONSUMER_RELOCATED_BUILD_DIR="${WORK_ROOT}/consumer-relocated-build"
CONSUMER_COLLISION_DIR="${WORK_ROOT}/consumer-collision"
CONSUMER_COLLISION_BUILD_DIR="${WORK_ROOT}/consumer-collision-build"
CONSUMER_XML_PATH="${DTDYNAMICS_INSTALL_VALIDATE_XML:-${CONSUMER_DIR}/install-consumer-model.xml}"
CONSUMER_PREINCLUDE_CONFIG="${DTDYNAMICS_INSTALL_VALIDATE_PREINCLUDE_CONFIG:-}"
EXPECTED_VERSION="${DTDYNAMICS_EXPECTED_VERSION:-0.1.3}"

if [[ -z "${INSTALL_LIBDIR}" || "${INSTALL_LIBDIR}" = /* || "${INSTALL_LIBDIR}" == *..* ]]; then
    echo "DTDYNAMICS_INSTALL_LIBDIR must be a relative install layout without '..': ${INSTALL_LIBDIR}" >&2
    exit 1
fi
if [[ -z "${INSTALL_DOCDIR}" || "${INSTALL_DOCDIR}" = /* || "${INSTALL_DOCDIR}" == *..* ]]; then
    echo "DTDYNAMICS_INSTALL_DOCDIR must be a relative install layout without '..': ${INSTALL_DOCDIR}" >&2
    exit 1
fi

if [[ ! -f "${DTMATH_INCLUDE_DIR}/dtMath/dtMath.h" ]]; then
    echo "dtMath headers not found at ${DTMATH_INCLUDE_DIR}" >&2
    echo "Set DTDYNAMICS_DTMATH_INCLUDE_DIR to the include root containing dtMath/dtMath.h." >&2
    exit 1
fi

cmake --preset release -S "${DTDYNAMICS_ROOT}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_INSTALL_LIBDIR="${INSTALL_LIBDIR}" \
    -DCMAKE_INSTALL_DOCDIR="${INSTALL_DOCDIR}" \
    -DDTDYNAMICS_DTMATH_INCLUDE_DIR="${DTMATH_INCLUDE_DIR}"
cmake --build "${BUILD_DIR}" --target install -- -j2

CONFIG_DIR="${INSTALL_PREFIX}/${INSTALL_LIBDIR}/cmake/dtDynamics"
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

DOC_DIR="${INSTALL_PREFIX}/${INSTALL_DOCDIR}"
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

python3 "${SCRIPT_DIR}/validate_installed_guide_provenance.py" --docs-root "${DOC_DIR}"
echo "PASS: installed guide source/PDF provenance validated."

mkdir -p "${CONSUMER_DIR}"
cat > "${CONSUMER_DIR}/CMakeLists.txt" <<'CONSUMER_CMAKE'
cmake_minimum_required(VERSION 3.16)
project(dtDynamicsInstallConsumer LANGUAGES CXX)

if(NOT DEFINED DTDYNAMICS_EXPECTED_VERSION)
    message(FATAL_ERROR "DTDYNAMICS_EXPECTED_VERSION must be provided by the validation driver")
endif()
if(NOT DEFINED DTDYNAMICS_PACKAGE_DIR)
    message(FATAL_ERROR "DTDYNAMICS_PACKAGE_DIR must be provided by the validation driver")
endif()
if(NOT DEFINED DTDYNAMICS_EXPECTED_PREFIX)
    message(FATAL_ERROR "DTDYNAMICS_EXPECTED_PREFIX must be provided by the validation driver")
endif()
if(NOT DEFINED DTDYNAMICS_EXPECTED_LIBRARY_DIR)
    message(FATAL_ERROR "DTDYNAMICS_EXPECTED_LIBRARY_DIR must be provided by the validation driver")
endif()

find_package(dtDynamics ${DTDYNAMICS_EXPECTED_VERSION} EXACT CONFIG REQUIRED
    PATHS "${DTDYNAMICS_PACKAGE_DIR}" NO_DEFAULT_PATH)
file(REAL_PATH "${DTDYNAMICS_PACKAGE_DIR}" _DTDYNAMICS_EXPECTED_PACKAGE_DIR)
file(REAL_PATH "${dtDynamics_DIR}" _DTDYNAMICS_FIRST_PACKAGE_DIR)
if(NOT "${_DTDYNAMICS_FIRST_PACKAGE_DIR}" STREQUAL
       "${_DTDYNAMICS_EXPECTED_PACKAGE_DIR}")
    message(FATAL_ERROR
        "dtDynamics resolved outside the selected staged package: "
        "${_DTDYNAMICS_FIRST_PACKAGE_DIR} != ${_DTDYNAMICS_EXPECTED_PACKAGE_DIR}")
endif()
file(REAL_PATH "${DTDYNAMICS_EXPECTED_PREFIX}" _DTDYNAMICS_EXPECTED_PREFIX)
file(REAL_PATH "${DTDYNAMICS_EXPECTED_LIBRARY_DIR}" _DTDYNAMICS_EXPECTED_LIBRARY_DIR)
function(_dtdynamics_require_within_selected_prefix path label)
    file(REAL_PATH "${path}" _DTDYNAMICS_REAL_PATH)
    file(RELATIVE_PATH _DTDYNAMICS_RELATIVE_PATH
        "${_DTDYNAMICS_EXPECTED_PREFIX}" "${_DTDYNAMICS_REAL_PATH}")
    if(_DTDYNAMICS_RELATIVE_PATH STREQUAL ".." OR
       _DTDYNAMICS_RELATIVE_PATH MATCHES "^\\.\\./")
        message(FATAL_ERROR
            "${label} resolved outside the selected staged prefix: ${_DTDYNAMICS_REAL_PATH}")
    endif()
endfunction()
function(_dtdynamics_require_within_selected_library_dir path label)
    file(REAL_PATH "${path}" _DTDYNAMICS_REAL_PATH)
    file(RELATIVE_PATH _DTDYNAMICS_RELATIVE_PATH
        "${_DTDYNAMICS_EXPECTED_LIBRARY_DIR}" "${_DTDYNAMICS_REAL_PATH}")
    if(_DTDYNAMICS_RELATIVE_PATH STREQUAL ".." OR
       _DTDYNAMICS_RELATIVE_PATH MATCHES "^\\.\\./")
        message(FATAL_ERROR
            "${label} resolved outside the selected staged library layout: ${_DTDYNAMICS_REAL_PATH}")
    endif()
endfunction()
_dtdynamics_require_within_selected_prefix("${dtDynamics_DIR}" "dtDynamics_DIR")

get_target_property(_DTDYNAMICS_FIRST_INTERFACE_INCLUDES
    dt::Dynamics INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(_DTDYNAMICS_IMPORTED_LOCATION
    dt::Dynamics IMPORTED_LOCATION_RELEASE)
if(NOT _DTDYNAMICS_IMPORTED_LOCATION OR
   _DTDYNAMICS_IMPORTED_LOCATION STREQUAL "_DTDYNAMICS_IMPORTED_LOCATION-NOTFOUND")
    get_target_property(_DTDYNAMICS_IMPORTED_LOCATION
        dt::Dynamics IMPORTED_LOCATION_NOCONFIG)
endif()
if(NOT _DTDYNAMICS_IMPORTED_LOCATION OR
   _DTDYNAMICS_IMPORTED_LOCATION STREQUAL "_DTDYNAMICS_IMPORTED_LOCATION-NOTFOUND")
    message(FATAL_ERROR "dt::Dynamics did not expose an imported release location")
endif()
_dtdynamics_require_within_selected_library_dir(
    "${_DTDYNAMICS_IMPORTED_LOCATION}" "dt::Dynamics imported location")

file(REAL_PATH "${DTDYNAMICS_EXPECTED_PREFIX}/include" _DTDYNAMICS_EXPECTED_INCLUDE)
set(_DTDYNAMICS_HAS_SELECTED_INCLUDE FALSE)
foreach(_DTDYNAMICS_INCLUDE IN LISTS _DTDYNAMICS_FIRST_INTERFACE_INCLUDES)
    file(REAL_PATH "${_DTDYNAMICS_INCLUDE}" _DTDYNAMICS_REAL_INCLUDE)
    if("${_DTDYNAMICS_REAL_INCLUDE}" STREQUAL "${_DTDYNAMICS_EXPECTED_INCLUDE}")
        set(_DTDYNAMICS_HAS_SELECTED_INCLUDE TRUE)
    endif()
endforeach()
if(NOT _DTDYNAMICS_HAS_SELECTED_INCLUDE)
    message(FATAL_ERROR
        "dt::Dynamics does not expose the selected staged include directory: "
        "${_DTDYNAMICS_EXPECTED_INCLUDE}")
endif()

# A consumer can resolve the package more than once; the package-owned target
# marker must accept the already-imported target rather than treat it as a collision.
find_package(dtDynamics ${DTDYNAMICS_EXPECTED_VERSION} EXACT CONFIG REQUIRED
    PATHS "${DTDYNAMICS_PACKAGE_DIR}" NO_DEFAULT_PATH)
file(REAL_PATH "${dtDynamics_DIR}" _DTDYNAMICS_SECOND_PACKAGE_DIR)
if(NOT "${_DTDYNAMICS_SECOND_PACKAGE_DIR}" STREQUAL
       "${_DTDYNAMICS_FIRST_PACKAGE_DIR}")
    message(FATAL_ERROR "Repeated dtDynamics lookup changed the resolved package directory")
endif()
get_target_property(_DTDYNAMICS_SECOND_INTERFACE_INCLUDES dt::Dynamics INTERFACE_INCLUDE_DIRECTORIES)
if(NOT "${_DTDYNAMICS_SECOND_INTERFACE_INCLUDES}" STREQUAL
       "${_DTDYNAMICS_FIRST_INTERFACE_INCLUDES}")
    message(FATAL_ERROR
        "Repeated dtDynamics lookup changed INTERFACE_INCLUDE_DIRECTORIES: "
        "${_DTDYNAMICS_FIRST_INTERFACE_INCLUDES} -> ${_DTDYNAMICS_SECOND_INTERFACE_INCLUDES}")
endif()
if(NOT "${dtDynamics_VERSION}" STREQUAL "${DTDYNAMICS_EXPECTED_VERSION}")
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
    -DDTDYNAMICS_PACKAGE_DIR="${CONFIG_DIR}" \
    -DDTDYNAMICS_EXPECTED_PREFIX="${INSTALL_PREFIX}" \
    -DDTDYNAMICS_EXPECTED_LIBRARY_DIR="${INSTALL_PREFIX}/${INSTALL_LIBDIR}" \
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
    -DDTDYNAMICS_PACKAGE_DIR="${CONFIG_DIR}" \
    -DDTDYNAMICS_EXPECTED_PREFIX="${INSTALL_PREFIX}" \
    -DDTDYNAMICS_EXPECTED_LIBRARY_DIR="${INSTALL_PREFIX}/${INSTALL_LIBDIR}" \
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

# An existing unrelated target with the public exported name must not be
# silently accepted, because it can bind the consumer to a different package.
mkdir -p "${CONSUMER_COLLISION_DIR}"
cat > "${CONSUMER_COLLISION_DIR}/CMakeLists.txt" <<'CONSUMER_COLLISION_CMAKE'
cmake_minimum_required(VERSION 3.16)
project(dtDynamicsInstallTargetCollision LANGUAGES CXX)

if(NOT DEFINED DTDYNAMICS_PACKAGE_DIR)
    message(FATAL_ERROR "DTDYNAMICS_PACKAGE_DIR must be provided by the validation driver")
endif()
add_library(dt::Dynamics INTERFACE IMPORTED GLOBAL)
find_package(dtDynamics ${DTDYNAMICS_EXPECTED_VERSION} EXACT CONFIG REQUIRED
    PATHS "${DTDYNAMICS_PACKAGE_DIR}" NO_DEFAULT_PATH)
message(FATAL_ERROR "dtDynamics package accepted a pre-existing dt::Dynamics target")
CONSUMER_COLLISION_CMAKE
if cmake -S "${CONSUMER_COLLISION_DIR}" -B "${CONSUMER_COLLISION_BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DDTDYNAMICS_PACKAGE_DIR="${CONFIG_DIR}" \
    -DDTDYNAMICS_DTMATH_INCLUDE_DIR="${DTMATH_INCLUDE_DIR}" \
    -DDTDYNAMICS_EXPECTED_VERSION="${EXPECTED_VERSION}" \
    > "${WORK_ROOT}/consumer-collision.log" 2>&1; then
    echo "Installed package accepted a pre-existing dt::Dynamics target." >&2
    exit 1
fi
if ! grep -Fq "dtDynamics target collision" "${WORK_ROOT}/consumer-collision.log"; then
    echo "Installed package collision test failed for an unexpected reason." >&2
    exit 1
fi
echo "PASS: installed dt::Dynamics target collision rejected."

if [[ -e "${RELOCATED_PREFIX}" ]]; then
    echo "Relocated validation prefix already exists: ${RELOCATED_PREFIX}" >&2
    exit 1
fi
mv "${INSTALL_PREFIX}" "${RELOCATED_PREFIX}"
RELOCATED_CONFIG_DIR="${RELOCATED_PREFIX}/${INSTALL_LIBDIR}/cmake/dtDynamics"
RELOCATED_DOC_DIR="${RELOCATED_PREFIX}/${INSTALL_DOCDIR}"
if [[ ! -f "${RELOCATED_CONFIG_DIR}/dtDynamicsTargets.cmake" ]]; then
    echo "Relocated dtDynamicsTargets.cmake was not found under ${RELOCATED_CONFIG_DIR}" >&2
    exit 1
fi
python3 "${SCRIPT_DIR}/validate_installed_guide_provenance.py" --docs-root "${RELOCATED_DOC_DIR}"
echo "PASS: relocated guide source/PDF provenance validated."
cmake -S "${CONSUMER_DIR}" -B "${CONSUMER_RELOCATED_BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DDTDYNAMICS_PACKAGE_DIR="${RELOCATED_CONFIG_DIR}" \
    -DDTDYNAMICS_EXPECTED_PREFIX="${RELOCATED_PREFIX}" \
    -DDTDYNAMICS_EXPECTED_LIBRARY_DIR="${RELOCATED_PREFIX}/${INSTALL_LIBDIR}" \
    -DDTDYNAMICS_DTMATH_INCLUDE_DIR="${DTMATH_INCLUDE_DIR}" \
    -DDTDYNAMICS_EXPECTED_VERSION="${EXPECTED_VERSION}"
cmake --build "${CONSUMER_RELOCATED_BUILD_DIR}" -- -j2
"${CONSUMER_RELOCATED_BUILD_DIR}/dtDynamicsInstallConsumer"
echo "PASS: relocated installed consumer executed."

echo "PASS: dtDynamics install/export validation succeeded."
echo "Install libdir: ${INSTALL_LIBDIR}"
echo "Relocated install prefix: ${RELOCATED_PREFIX}"
