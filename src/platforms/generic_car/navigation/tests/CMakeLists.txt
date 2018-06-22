# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
add_executable(test_cfield_search test_cfield_search.cpp)
target_link_libraries(test_cfield_search nav field fastplot utilities)

add_executable(test_rmap_search test_rmap_search.cpp)
target_link_libraries(test_rmap_search nav decomp road lightviz utilities)

add_executable(test_traffic_flow test_traffic_flow.cpp)
target_link_libraries(test_traffic_flow nav decomp road lightviz utilities)

add_executable(test_isodist test_isodistance.cpp)
target_link_libraries(test_isodist nav decomp road lightviz utilities)