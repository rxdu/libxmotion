# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(LOCAL_PLANNER_LIB_SRC
    local_planner.cpp
)
add_library(local_planner STATIC ${LOCAL_PLANNER_LIB_SRC})
target_link_libraries(local_planner ompl rrtstarkd)

# Add executables
add_executable(test_local test_local.cpp)
target_link_libraries(test_local local_planner)
