if (ENABLE_TBOT_APP)
  add_subdirectory(tbot)
endif ()

if (ENABLE_SWERVEBOT_APP)
  add_subdirectory(swervebot)
endif ()

if (ENABLE_QUADRUPED_APP)
  add_subdirectory(quadruped)
endif ()