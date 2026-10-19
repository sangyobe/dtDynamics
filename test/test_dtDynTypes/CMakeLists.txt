cmake_minimum_required(VERSION 3.12)

project(test_dtDynTypes)

add_executable(${PROJECT_NAME}
    test_dtDynTypes.cpp
)

# Release validation must execute the semantic assertions in this test target.
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /UNDEBUG)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -UNDEBUG)
endif()

target_link_libraries(${PROJECT_NAME}
    PRIVATE dt::Dynamics
)

# The allocation-free proof also counts direct malloc/calloc/realloc calls (dtMath dense
# storage). GNU ld and lld support --wrap; other toolchains count operator new only.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_compile_definitions(${PROJECT_NAME} PRIVATE DTDYN_TEST_WRAP_C_ALLOCATOR)
    target_link_options(${PROJECT_NAME} PRIVATE
        -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=realloc)
endif()

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
