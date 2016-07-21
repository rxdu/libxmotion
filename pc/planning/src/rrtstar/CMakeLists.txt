# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(RRTSTARKD_LIB_SRC
    rrtstar_kd.cpp
)
add_library(rrtstarkd STATIC ${RRTSTARKD_LIB_SRC})
target_link_libraries(rrtstarkd ompl)

# Add executables
add_executable(test_rrts test_rrts.cpp)
target_link_libraries(test_rrts ompl rrtstarkd)
