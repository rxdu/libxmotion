# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_polygon test_polygon.cpp)
target_link_libraries(test_polygon geometry)

add_executable(test_cspline test_cspline.cpp)
target_link_libraries(test_cspline geometry)

add_executable(test_pcurve test_pcurve.cpp)
target_link_libraries(test_pcurve geometry)

add_executable(test_polynomial test_polynomial.cpp)
target_link_libraries(test_polynomial geometry)

add_executable(test_polyline_draw test_polyline_draw.cpp )
target_link_libraries(test_polyline_draw geometry)

add_executable(test_pcurve_draw test_pcurve_draw.cpp)
target_link_libraries(test_pcurve_draw geometry)

add_executable(test_cspline_draw test_cspline_draw.cpp )
target_link_libraries(test_cspline_draw geometry)

add_executable(test_polygon_draw test_polygon_draw.cpp )
target_link_libraries(test_polygon_draw geometry)

add_executable(test_dist_draw test_dist_draw.cpp )
target_link_libraries(test_dist_draw geometry)