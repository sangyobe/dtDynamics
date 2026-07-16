cmake_minimum_required(VERSION 3.12)

project(benchmark_dtDynamics)

add_executable(${PROJECT_NAME}
    benchmark_dtDynamics.cpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE dt::Dynamics
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE DTDYNAMICS_BENCHMARK_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
)

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
