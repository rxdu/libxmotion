# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
#set(TESTCASES_SRC
#    graph_unittest.cpp
#)
#add_library(testcases STATIC ${TESTCASES_SRC})
#target_link_libraries(qtree ${OpenCV_LIBS})

# Add executables
set(TESTCASES_SRC
    graph_unittest.cpp
    typesize_unittest.cpp
    maputils_unittest.cpp
)
add_executable(gtest_all start_tests.cpp ${TESTCASES_SRC})
target_link_libraries(gtest_all gtest map ${OpenCV_LIBS})

add_executable(gtest_maputils start_tests.cpp maputils_unittest.cpp)
target_link_libraries(gtest_maputils gtest map ${OpenCV_LIBS})