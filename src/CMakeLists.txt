## Add source directories
# Application
add_subdirectory(apps)

# Modules
add_subdirectory(interface)
add_subdirectory(driver)
add_subdirectory(mapping)
add_subdirectory(planning)
add_subdirectory(control)
add_subdirectory(utilities)

if(ENABLE_TARGET_DESKTOP)
add_subdirectory(visualization)
endif()