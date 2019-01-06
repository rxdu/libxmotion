# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_image_viz test_image_viz.cpp)
target_link_libraries(test_image_viz lightviz)

add_executable(test_sgrid_viz test_sgrid_viz.cpp )
target_link_libraries(test_sgrid_viz lightviz)

add_executable(test_polygon_viz test_polygon_viz.cpp )
target_link_libraries(test_polygon_viz lightviz)

add_executable(test_cspline_viz test_cspline_viz.cpp )
target_link_libraries(test_cspline_viz lightviz)

add_executable(test_pcurve_viz test_pcurve_viz.cpp)
target_link_libraries(test_pcurve_viz lightviz)

# add_executable(test_dist_viz test_dist_viz.cpp )
# target_link_libraries(test_dist_viz lightviz)

# add_executable(test_geogrid_viz test_geogrid_viz.cpp)
# target_link_libraries(test_geogrid_viz lightviz)

# add_executable(test_rrt_viz test_rrt_viz.cpp)
# target_link_libraries(test_rrt_viz lightviz)