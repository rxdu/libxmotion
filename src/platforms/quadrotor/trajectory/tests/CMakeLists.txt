# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
add_executable(test_jointcorridor test_jointcorridor.cpp)
target_link_libraries(test_jointcorridor quad_traj)
