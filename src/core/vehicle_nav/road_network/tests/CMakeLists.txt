# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(lanelet_demo lanelet_demo.cpp)
target_link_libraries(lanelet_demo lanelet)

add_executable(lanelet_test lanelet_test.cpp)
target_link_libraries(lanelet_test lanelet)

add_executable(test_lanelet_ref test_lanelet_ref.cpp)
target_link_libraries(test_lanelet_ref lanelet)

add_executable(test_roadmap test_roadmap.cpp)
target_link_libraries(test_roadmap road lightviz utilities)

add_executable(test_coordinate test_coordinate.cpp)
target_link_libraries(test_coordinate road)

add_executable(test_roadmap_geometry test_roadmap_geometry.cpp)
target_link_libraries(test_roadmap_geometry road lightviz utilities)