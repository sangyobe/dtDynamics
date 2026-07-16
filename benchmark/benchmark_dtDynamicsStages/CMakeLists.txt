cmake_minimum_required(VERSION 3.12)

project(benchmark_dtDynamicsStages)

add_executable(${PROJECT_NAME}
    benchmark_dtDynamicsStages.cpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE dt::Dynamics
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE DTDYNAMICS_BENCHMARK_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
)

target_include_directories(${PROJECT_NAME}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..
)

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
