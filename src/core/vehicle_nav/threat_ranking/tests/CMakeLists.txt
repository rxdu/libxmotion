# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_topogeo_graph test_topogeo_graph.cpp)
target_link_libraries(test_topogeo_graph rank lightviz utilities)

add_executable(test_threat_model test_threat_model.cpp)
target_link_libraries(test_threat_model road rank tfield lightviz utilities)