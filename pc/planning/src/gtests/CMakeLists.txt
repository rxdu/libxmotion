# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
#set(TESTCASES_SRC
#    graph_unittest.cpp
#)
#add_library(testcases STATIC ${TESTCASES_SRC})
#target_link_libraries(qtree ${OpenCV_LIBS})

# Add executables
set(TESTCASES_SRC
    graph_unittest.cpp
)
add_executable(start_tests start_tests.cpp ${TESTCASES_SRC})
target_link_libraries(start_tests gtest)
