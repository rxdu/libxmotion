# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
set(PLANNER_LIB_SRC
    rrts_planner.cpp
    #graph_planner.cpp
    quad_planner.cpp
    planner_utils.cpp
)
add_library(planner STATIC ${PLANNER_LIB_SRC})
target_link_libraries(planner ompl rrtstarkd ${OpenCV_LIBS})

# Add executables
add_executable(test_local test_local.cpp)
target_link_libraries(test_local planner)

add_executable(test_global test_global.cpp)
target_link_libraries(test_global map planner qtree)
