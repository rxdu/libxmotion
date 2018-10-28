# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
add_executable(test_i2c_readreg i2c_readreg.c)
target_link_libraries(test_i2c_readreg tools)

add_executable(test_funcs test_funcs.cpp)
target_link_libraries(test_funcs tools)

add_executable(test_i2c_readreg2 i2c_readreg2.c)
target_link_libraries(test_i2c_readreg2 tools)
