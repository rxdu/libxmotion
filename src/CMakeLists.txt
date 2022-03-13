## Add source directories
# Application
# add_subdirectory(dview)

# Modules
add_subdirectory(interface)
add_subdirectory(driver)
add_subdirectory(mapping)
add_subdirectory(planning)
add_subdirectory(control)
add_subdirectory(utilities)
add_subdirectory(visualization)

if(BUILD_TESTS)
    add_subdirectory(testing)
endif()
