# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(app_traffic_viewer app_traffic_viewer.cpp)
target_link_libraries(app_traffic_viewer lightview traffic_viewer)

add_executable(app_traffic_viewer_loop app_traffic_viewer_loop.cpp)
target_link_libraries(app_traffic_viewer_loop lightview traffic_viewer)