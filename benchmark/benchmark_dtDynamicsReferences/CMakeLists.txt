cmake_minimum_required(VERSION 3.12)

project(benchmark_dtDynamicsReferences)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 OLD)
endif()
set(COAL_DISABLE_HPP_FCL_WARNINGS ON CACHE BOOL
    "Disable hpp-fcl compatibility warnings in Pinocchio validation dependency configs" FORCE)

if(DEFINED ENV{DTDYNAMICS_MUJOCO_ROOT} AND NOT "$ENV{DTDYNAMICS_MUJOCO_ROOT}" STREQUAL "")
    set(_DTDYNAMICS_MUJOCO_ROOT_DEFAULT "$ENV{DTDYNAMICS_MUJOCO_ROOT}")
else()
    set(_DTDYNAMICS_MUJOCO_ROOT_DEFAULT
        "${DTDYNAMICS_VALIDATION_DEPS_DIR}/mujoco-${DTDYNAMICS_MUJOCO_VERSION}")
endif()
set(DTDYNAMICS_MUJOCO_ROOT "${_DTDYNAMICS_MUJOCO_ROOT_DEFAULT}" CACHE PATH
    "Private MuJoCo ${DTDYNAMICS_MUJOCO_VERSION} root used only by dtDynamics reference benchmarks")

find_path(MUJOCO_INCLUDE_DIR
    NAMES mujoco/mujoco.h
    PATHS ${DTDYNAMICS_MUJOCO_ROOT}/include
    NO_DEFAULT_PATH
)

find_library(MUJOCO_LIBRARY
    NAMES mujoco
    PATHS ${DTDYNAMICS_MUJOCO_ROOT}/lib ${DTDYNAMICS_MUJOCO_ROOT}/lib64
    NO_DEFAULT_PATH
)

if(DEFINED ENV{DTDYNAMICS_PINOCCHIO_ROOT} AND NOT "$ENV{DTDYNAMICS_PINOCCHIO_ROOT}" STREQUAL "")
    set(_DTDYNAMICS_PINOCCHIO_ROOT_DEFAULT "$ENV{DTDYNAMICS_PINOCCHIO_ROOT}")
else()
    set(_DTDYNAMICS_PINOCCHIO_ROOT_DEFAULT
        "${DTDYNAMICS_VALIDATION_DEPS_DIR}/pinocchio-${DTDYNAMICS_PINOCCHIO_VERSION}")
endif()
set(DTDYNAMICS_PINOCCHIO_ROOT "${_DTDYNAMICS_PINOCCHIO_ROOT_DEFAULT}" CACHE PATH
    "Private Pinocchio ${DTDYNAMICS_PINOCCHIO_VERSION} root used only by dtDynamics reference benchmarks")

list(PREPEND CMAKE_PREFIX_PATH "${DTDYNAMICS_PINOCCHIO_ROOT}")
find_package(pinocchio CONFIG QUIET
    PATHS ${DTDYNAMICS_PINOCCHIO_ROOT}
    NO_DEFAULT_PATH
)

if(DEFINED pinocchio_VERSION AND
   NOT pinocchio_VERSION VERSION_EQUAL DTDYNAMICS_PINOCCHIO_VERSION)
    message(STATUS
        "Skipping ${PROJECT_NAME}: expected Pinocchio ${DTDYNAMICS_PINOCCHIO_VERSION}, "
        "found ${pinocchio_VERSION} at ${DTDYNAMICS_PINOCCHIO_ROOT}.")
    return()
endif()

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY OR NOT pinocchio_FOUND)
    message(STATUS
        "Skipping ${PROJECT_NAME}: MuJoCo ${DTDYNAMICS_MUJOCO_VERSION} and Pinocchio "
        "${DTDYNAMICS_PINOCCHIO_VERSION} are required for reference benchmarks. "
        "Run tools/install_validation_deps.sh --all from the dtDynamics package root.")
    return()
endif()

get_filename_component(MUJOCO_LIBRARY_DIR "${MUJOCO_LIBRARY}" DIRECTORY)
set(_DTDYNAMICS_REFERENCE_RPATH
    "${MUJOCO_LIBRARY_DIR}"
    "${DTDYNAMICS_PINOCCHIO_ROOT}/lib"
    "${DTDYNAMICS_PINOCCHIO_ROOT}/lib64"
)

add_executable(${PROJECT_NAME}
    benchmark_dtDynamicsReferences.cpp
)

target_include_directories(${PROJECT_NAME}
    PRIVATE ${MUJOCO_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    dt::Dynamics
    ${MUJOCO_LIBRARY}
    pinocchio::pinocchio
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
    COAL_DISABLE_HPP_FCL_WARNINGS
    DTDYNAMICS_BENCHMARK_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    BUILD_RPATH "${_DTDYNAMICS_REFERENCE_RPATH}"
    INSTALL_RPATH "${_DTDYNAMICS_REFERENCE_RPATH}"
)
