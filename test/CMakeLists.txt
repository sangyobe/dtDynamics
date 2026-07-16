add_subdirectory(test_dtDynTypes)

if(BUILD_DTDYNAMICS_MUJOCO_TEST)
    add_subdirectory(test_dtDynamicsMujocoRef)
endif()

if(BUILD_DTDYNAMICS_PINOCCHIO_TEST)
    add_subdirectory(test_dtDynamicsPinocchioRef)
endif()
