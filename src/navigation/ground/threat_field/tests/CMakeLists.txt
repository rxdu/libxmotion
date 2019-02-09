# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

################### 

add_executable(test_field_basis test_field_basis.cpp)
target_link_libraries(test_field_basis tfield threatdist navviz utils)

add_executable(test_threat_map test_threat_map.cpp)
target_link_libraries(test_threat_map tfield threatdist navviz utils)

add_executable(test_derived_prob test_derived_prob.cpp)
target_link_libraries(test_derived_prob reachability utils)

add_executable(test_derived_prob2 test_derived_prob2.cpp)
target_link_libraries(test_derived_prob2 reachability utils)