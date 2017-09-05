# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
set(PLANNER_LIB_SRC
    rrts_planner.cpp
    rrts_planner_local.cpp
    #graph_planner.cpp
    quad_planner.cpp
)
add_library(planner STATIC ${PLANNER_LIB_SRC})
target_link_libraries(planner geometry map ompl rrtstarkd ${OpenCV_LIBS} ${Octomap_LIBS} ${LCM_LIBRARIES})

# Add executables
add_executable(test_local tests/test_local.cpp)
target_link_libraries(test_local planner)

add_executable(test_global tests/test_global.cpp)
target_link_libraries(test_global map planner geometry)

add_executable(test_rrtvis tests/test_rrtvis.cpp)
target_link_libraries(test_rrtvis map vis planner ${OpenCV_LIBS})

add_executable(test_planner tests/test_planner.cpp)
target_link_libraries(test_planner map vis planner ${OpenCV_LIBS})
