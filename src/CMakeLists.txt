# cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/src/cmake/modules")

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

## Add sub source directories
# Core packages of librav
add_subdirectory(kernel)
add_subdirectory(third_party)

# Packages built on top of kernel
add_subdirectory(mcontrol)
add_subdirectory(navigation)
add_subdirectory(platforms)
add_subdirectory(viewers)

# Publication-related code
add_subdirectory(publication)

## Add test source directories
if(BUILD_TESTS)
    add_subdirectory(unit_tests) 
endif()
