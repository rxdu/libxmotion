## Add source directories
# Robots
add_subdirectory(platform)

# Modules
add_subdirectory(interface)
# add_subdirectory(ipc)l
add_subdirectory(driver)
add_subdirectory(mapping)
add_subdirectory(estimation)
add_subdirectory(control)
add_subdirectory(planning)
add_subdirectory(utilities)

if (ENABLE_TARGET_DESKTOP)
  add_subdirectory(visualization)
endif ()