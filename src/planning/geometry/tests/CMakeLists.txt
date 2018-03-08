## Config common include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add executables
add_executable(test_squaregrid test_squaregrid.cpp)
target_link_libraries(test_squaregrid geometry)
