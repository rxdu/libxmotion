# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
add_executable(test_vfield_sim test_vehiclefield_sim.cpp)
target_link_libraries(test_vfield_sim field traffic ${LCM_LIBRARIES})

add_executable(test_cfield_sim test_collifield_sim.cpp)
target_link_libraries(test_cfield_sim field traffic ${LCM_LIBRARIES})