# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_sgrid_draw test_sgrid_draw.cpp )
target_link_libraries(test_sgrid_draw coreviz)

add_executable(test_polyline_draw test_polyline_draw.cpp )
target_link_libraries(test_polyline_draw coreviz)

add_executable(test_pcurve_draw test_pcurve_draw.cpp)
target_link_libraries(test_pcurve_draw coreviz)

add_executable(test_cspline_draw test_cspline_draw.cpp )
target_link_libraries(test_cspline_draw coreviz)

add_executable(test_polygon_draw test_polygon_draw.cpp )
target_link_libraries(test_polygon_draw coreviz)

add_executable(test_dist_draw test_dist_draw.cpp )
target_link_libraries(test_dist_draw coreviz)

add_executable(test_cgrid_draw test_cgrid_draw.cpp)
target_link_libraries(test_cgrid_draw coreviz)

add_executable(test_rrt_draw test_rrt_draw.cpp)
target_link_libraries(test_rrt_draw coreviz)
