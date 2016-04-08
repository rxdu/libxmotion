# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/remoteApi)
#include_directories(${CMAKE_SOURCE_DIR}/src)

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(VREP_REMOTE_LIB_SRC
    remoteApi/extApi.c
    remoteApi/extApiCustom.c
    remoteApi/extApiPlatform.c
)
add_library(vrep_remote_api STATIC ${VREP_REMOTE_LIB_SRC})
add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255)
#target_link_libraries(vis ${OpenCV_LIBS})

# Add executables
#add_executable(test_graph test_graph.cpp)
#target_link_libraries(test_graph qtree vis map ${OpenCV_LIBS})
