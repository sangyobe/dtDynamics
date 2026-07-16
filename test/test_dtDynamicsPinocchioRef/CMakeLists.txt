cmake_minimum_required(VERSION 3.12)

project(test_dtDynamicsPinocchioRef)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 OLD)
endif()
set(COAL_DISABLE_HPP_FCL_WARNINGS ON CACHE BOOL
    "Disable hpp-fcl compatibility warnings in Pinocchio validation dependency configs" FORCE)

if(DEFINED ENV{DTDYNAMICS_PINOCCHIO_ROOT} AND NOT "$ENV{DTDYNAMICS_PINOCCHIO_ROOT}" STREQUAL "")
    set(_DTDYNAMICS_PINOCCHIO_ROOT_DEFAULT "$ENV{DTDYNAMICS_PINOCCHIO_ROOT}")
else()
    set(_DTDYNAMICS_PINOCCHIO_ROOT_DEFAULT
        "${DTDYNAMICS_VALIDATION_DEPS_DIR}/pinocchio-${DTDYNAMICS_PINOCCHIO_VERSION}")
endif()
set(DTDYNAMICS_PINOCCHIO_ROOT "${_DTDYNAMICS_PINOCCHIO_ROOT_DEFAULT}" CACHE PATH
    "Private Pinocchio ${DTDYNAMICS_PINOCCHIO_VERSION} root used only by dtDynamics validation tests")

list(PREPEND CMAKE_PREFIX_PATH "${DTDYNAMICS_PINOCCHIO_ROOT}")
find_package(pinocchio CONFIG QUIET
    PATHS ${DTDYNAMICS_PINOCCHIO_ROOT}
    NO_DEFAULT_PATH
)
if(NOT pinocchio_FOUND)
    message(STATUS
        "Skipping ${PROJECT_NAME}: Pinocchio ${DTDYNAMICS_PINOCCHIO_VERSION} validation dependency was not found. "
        "Set DTDYNAMICS_PINOCCHIO_ROOT to the private Pinocchio install root, "
        "or run tools/install_validation_deps.sh --pinocchio from the dtDynamics package root.")
    return()
endif()

if(DEFINED pinocchio_VERSION AND
   NOT pinocchio_VERSION VERSION_EQUAL DTDYNAMICS_PINOCCHIO_VERSION)
    message(STATUS
        "Skipping ${PROJECT_NAME}: expected Pinocchio ${DTDYNAMICS_PINOCCHIO_VERSION}, "
        "found ${pinocchio_VERSION} at ${DTDYNAMICS_PINOCCHIO_ROOT}.")
    return()
endif()

set(_DTDYNAMICS_PINOCCHIO_RPATH
    "${DTDYNAMICS_PINOCCHIO_ROOT}/lib"
    "${DTDYNAMICS_PINOCCHIO_ROOT}/lib64"
)

add_executable(${PROJECT_NAME}
    test_dtDynamicsPinocchioRef.cpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    dt::Dynamics
    pinocchio::pinocchio
)
target_compile_definitions(${PROJECT_NAME}
    PRIVATE COAL_DISABLE_HPP_FCL_WARNINGS
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    BUILD_RPATH "${_DTDYNAMICS_PINOCCHIO_RPATH}"
    INSTALL_RPATH "${_DTDYNAMICS_PINOCCHIO_RPATH}"
)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
