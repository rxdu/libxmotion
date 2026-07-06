## Add source directories
# Modules
add_subdirectory(types)
add_subdirectory(mapping)
add_subdirectory(decision)
add_subdirectory(estimation)
add_subdirectory(control)
add_subdirectory(planning)

if (ENABLE_VISUALIZATION)
  add_subdirectory(visualization)
endif ()
