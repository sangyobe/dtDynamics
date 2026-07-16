cmake_minimum_required(VERSION 3.12)

project(test_dtDynTypes)

add_executable(${PROJECT_NAME}
    test_dtDynTypes.cpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE dt::Dynamics
)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
