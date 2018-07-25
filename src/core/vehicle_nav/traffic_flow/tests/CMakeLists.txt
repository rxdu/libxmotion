# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
# add_executable(test_cfield_search test_cfield_search.cpp)
# target_link_libraries(test_cfield_search tflow field fastplot utilities)

add_executable(test_rmap_search test_rmap_search.cpp)
target_link_libraries(test_rmap_search tflow lightviz utilities)

add_executable(test_traffic_flow test_traffic_flow.cpp)
target_link_libraries(test_traffic_flow tflow lightviz utilities)

add_executable(test_roi test_roi.cpp)
target_link_libraries(test_roi tflow lightviz utilities)

add_executable(test_topogeo_graph test_topogeo_graph.cpp)
target_link_libraries(test_topogeo_graph tflow lightviz utilities)

