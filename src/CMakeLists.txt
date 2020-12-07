## Add source directories
# Application
# add_subdirectory(dview)

# Modules
add_subdirectory(modules)

# Unit test
if(BUILD_TESTS)
    add_subdirectory(unit_test)
endif()