#
# The root cmake script for dtDynamics, the native ART dynamics ingredient package.
#
# dtDynamics is a standalone dynamics ingredient package. MuJoCo and Pinocchio
# are allowed only in tests and benchmarks, not in the production library target.
#
cmake_minimum_required(VERSION 3.16)

project(
    dtDynamics
    LANGUAGES CXX
    VERSION 0.1.1
    DESCRIPTION "Standalone dynamics ingredient package for control and WBC"
)
set(PROJECT_NAMESPACE ${PROJECT_NAME})

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    set(DTDYNAMICS_IS_TOP_LEVEL ON)
else()
    set(DTDYNAMICS_IS_TOP_LEVEL OFF)
endif()

cmake_policy(VERSION 3.16)
cmake_policy(SET CMP0077 NEW)

if("$ENV{ARTF_INSTALL_DIR}" STREQUAL "")
    set(ARTF_INSTALL_DIR "$ENV{HOME}/.local/ART_Framework")
else()
    file(TO_CMAKE_PATH "$ENV{ARTF_INSTALL_DIR}" ARTF_INSTALL_DIR)
endif()
message(STATUS "ARTF_INSTALL_DIR: ${ARTF_INSTALL_DIR}")

set(CMAKE_PREFIX_PATH "${ARTF_INSTALL_DIR};${CMAKE_PREFIX_PATH}")
if(DTDYNAMICS_IS_TOP_LEVEL AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${ARTF_INSTALL_DIR}" CACHE PATH "ART Framework install prefix" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

set(DTDYNAMICS_BUILD_TEST_DEFAULT ${DTDYNAMICS_IS_TOP_LEVEL})
if(DTDYNAMICS_IS_TOP_LEVEL AND DEFINED BUILD_TEST)
    set(DTDYNAMICS_BUILD_TEST_DEFAULT ${BUILD_TEST})
endif()
set(DTDYNAMICS_BUILD_BENCHMARK_DEFAULT OFF)
if(DTDYNAMICS_IS_TOP_LEVEL AND DEFINED BUILD_BENCHMARK)
    set(DTDYNAMICS_BUILD_BENCHMARK_DEFAULT ${BUILD_BENCHMARK})
endif()
option(DTDYNAMICS_BUILD_TEST "Build tests for ${PROJECT_NAME}" ${DTDYNAMICS_BUILD_TEST_DEFAULT})
option(DTDYNAMICS_BUILD_BENCHMARK "Build benchmarks for ${PROJECT_NAME}" ${DTDYNAMICS_BUILD_BENCHMARK_DEFAULT})
option(DTDYNAMICS_BUILD_DOCS "Build Doxygen/Graphviz documentation for ${PROJECT_NAME}" OFF)
option(DTDYNAMICS_BUILD_REFERENCE_BENCHMARK "Build MuJoCo/Pinocchio reference benchmark executable" ON)
option(BUILD_DTDYNAMICS_MUJOCO_TEST "Build MuJoCo reference validation tests" ON)
option(BUILD_DTDYNAMICS_PINOCCHIO_TEST "Build Pinocchio reference validation tests" ON)
set(DTDYNAMICS_VALIDATION_DEPS_DIR
    "${PROJECT_SOURCE_DIR}/.validation_deps"
    CACHE PATH "Private dependency prefix used only by dtDynamics validation tests")
set(DTDYNAMICS_MUJOCO_VERSION "3.8.0" CACHE STRING "Pinned MuJoCo validation version")
set(DTDYNAMICS_PINOCCHIO_VERSION "3.9.0" CACHE STRING "Pinned Pinocchio validation version")

find_path(DTDYNAMICS_DTMATH_INCLUDE_DIR
    NAMES dtMath/dtMath.h
    HINTS "${ARTF_INSTALL_DIR}/include"
    DOC "dtMath include root used by the native dtDynamics package"
)
if(NOT DTDYNAMICS_DTMATH_INCLUDE_DIR)
    message(FATAL_ERROR
        "dtMath was not found. Install dtMath into ${ARTF_INSTALL_DIR}/include "
        "or configure DTDYNAMICS_DTMATH_INCLUDE_DIR to the include root containing dtMath/dtMath.h.")
endif()

set(DTDYNAMICS_SRC
    src/dtDynTypes.cpp
    src/dtDynSpec.cpp
    src/dtDynValidation.cpp
    src/dtDynModel.cpp
    src/dtDynData.cpp
    src/dtDynCompile.cpp
    src/dtDynMath.cpp
    src/dtDynUtils.cpp
    src/dtDynKinematics.cpp
    src/dtDynDynamics.cpp
)

add_library(${PROJECT_NAME} STATIC ${DTDYNAMICS_SRC})
add_library(dt::Dynamics ALIAS ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES EXPORT_NAME Dynamics)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_include_directories(${PROJECT_NAME}
    SYSTEM PUBLIC
        $<BUILD_INTERFACE:${DTDYNAMICS_DTMATH_INCLUDE_DIR}>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

if(DTDYNAMICS_BUILD_TEST)
    enable_testing()
    add_subdirectory(test)
endif()

if(DTDYNAMICS_BUILD_BENCHMARK)
    add_subdirectory(benchmark/benchmark_dtDynamics)
    add_subdirectory(benchmark/benchmark_dtDynamicsStages)
    if(DTDYNAMICS_BUILD_REFERENCE_BENCHMARK)
        add_subdirectory(benchmark/benchmark_dtDynamicsReferences)
    endif()
endif()

if(DTDYNAMICS_BUILD_DOCS)
    find_program(DOXYGEN_EXECUTABLE doxygen REQUIRED)
    find_program(GRAPHVIZ_DOT_EXECUTABLE dot REQUIRED)
    set(DTDYNAMICS_DOCS_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/docs/doxygen")
    add_custom_target(docs
        COMMAND ${CMAKE_COMMAND} -E env
            "PATH=$ENV{PATH}"
            "DTDYNAMICS_DOCS_OUTPUT_DIR=${DTDYNAMICS_DOCS_OUTPUT_DIR}"
            ${PROJECT_SOURCE_DIR}/tools/validate_docs.sh
        BYPRODUCTS ${DTDYNAMICS_DOCS_OUTPUT_DIR}/html/index.html
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        COMMENT "Generating dtDynamics Doxygen/Graphviz manual"
        VERBATIM
    )
    install(DIRECTORY ${DTDYNAMICS_DOCS_OUTPUT_DIR}/html/
        DESTINATION ${CMAKE_INSTALL_DOCDIR}/html
        OPTIONAL
    )
endif()

install(TARGETS ${PROJECT_NAME}
    EXPORT dtDynamicsTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/dtDynamics
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

if(EXISTS "${PROJECT_SOURCE_DIR}/docs")
    install(DIRECTORY docs/
        DESTINATION ${CMAKE_INSTALL_DOCDIR}
        REGEX "/guides/README\\.md$" EXCLUDE
        REGEX "/guides/themes(/|$)" EXCLUDE
        REGEX "/guides/(model_guide|usage_guide)/[^/]+\\.md$" EXCLUDE
    )
endif()

install(FILES
    ${PROJECT_SOURCE_DIR}/LICENSE
    ${PROJECT_SOURCE_DIR}/README.md
    DESTINATION ${CMAKE_INSTALL_DOCDIR}
)

install(EXPORT dtDynamicsTargets
    NAMESPACE dt::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/dtDynamics
)

configure_package_config_file(
    ${PROJECT_SOURCE_DIR}/cmake/dtDynamicsConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/dtDynamicsConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/dtDynamics
    PATH_VARS CMAKE_INSTALL_INCLUDEDIR
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/dtDynamicsConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/dtDynamicsConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/dtDynamicsConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/dtDynamics
)

message(STATUS "DTDYNAMICS_IS_TOP_LEVEL             : ${DTDYNAMICS_IS_TOP_LEVEL}")
message(STATUS "DTDYNAMICS_BUILD_TEST               : ${DTDYNAMICS_BUILD_TEST}")
message(STATUS "DTDYNAMICS_BUILD_BENCHMARK          : ${DTDYNAMICS_BUILD_BENCHMARK}")
message(STATUS "DTDYNAMICS_BUILD_DOCS               : ${DTDYNAMICS_BUILD_DOCS}")
message(STATUS "DTDYNAMICS_BUILD_REFERENCE_BENCHMARK: ${DTDYNAMICS_BUILD_REFERENCE_BENCHMARK}")
message(STATUS "BUILD_DTDYNAMICS_MUJOCO_TEST        : ${BUILD_DTDYNAMICS_MUJOCO_TEST}")
message(STATUS "BUILD_DTDYNAMICS_PINOCCHIO_TEST     : ${BUILD_DTDYNAMICS_PINOCCHIO_TEST}")
message(STATUS "DTDYNAMICS_VALIDATION_DEPS_DIR      : ${DTDYNAMICS_VALIDATION_DEPS_DIR}")
message(STATUS "DTDYNAMICS_MUJOCO_VERSION           : ${DTDYNAMICS_MUJOCO_VERSION}")
message(STATUS "DTDYNAMICS_PINOCCHIO_VERSION        : ${DTDYNAMICS_PINOCCHIO_VERSION}")
message(STATUS "DTDYNAMICS_DTMATH_INCLUDE_DIR       : ${DTDYNAMICS_DTMATH_INCLUDE_DIR}")
