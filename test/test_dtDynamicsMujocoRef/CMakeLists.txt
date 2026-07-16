cmake_minimum_required(VERSION 3.12)

project(test_dtDynamicsMujocoRef)

if(DEFINED ENV{DTDYNAMICS_MUJOCO_ROOT} AND NOT "$ENV{DTDYNAMICS_MUJOCO_ROOT}" STREQUAL "")
    set(_DTDYNAMICS_MUJOCO_ROOT_DEFAULT "$ENV{DTDYNAMICS_MUJOCO_ROOT}")
else()
    set(_DTDYNAMICS_MUJOCO_ROOT_DEFAULT
        "${DTDYNAMICS_VALIDATION_DEPS_DIR}/mujoco-${DTDYNAMICS_MUJOCO_VERSION}")
endif()
set(DTDYNAMICS_MUJOCO_ROOT "${_DTDYNAMICS_MUJOCO_ROOT_DEFAULT}" CACHE PATH
    "Private MuJoCo ${DTDYNAMICS_MUJOCO_VERSION} root used only by dtDynamics validation tests")

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

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
    message(STATUS
        "Skipping ${PROJECT_NAME}: MuJoCo ${DTDYNAMICS_MUJOCO_VERSION} validation dependency was not found. "
        "Set DTDYNAMICS_MUJOCO_ROOT to the private MuJoCo ${DTDYNAMICS_MUJOCO_VERSION} install root, "
        "or run tools/install_validation_deps.sh --mujoco from the dtDynamics package root.")
    return()
endif()

get_filename_component(MUJOCO_LIBRARY_DIR "${MUJOCO_LIBRARY}" DIRECTORY)

add_executable(${PROJECT_NAME}
    test_dtDynamicsMujocoRef.cpp
)

target_include_directories(${PROJECT_NAME}
    PRIVATE ${MUJOCO_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    dt::Dynamics
    ${MUJOCO_LIBRARY}
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    BUILD_RPATH "${MUJOCO_LIBRARY_DIR}"
    INSTALL_RPATH "${MUJOCO_LIBRARY_DIR}"
)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
