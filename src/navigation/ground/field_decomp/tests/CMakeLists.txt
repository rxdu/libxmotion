# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

################### 

add_executable(test_field_basis test_field_basis.cpp)
target_link_libraries(test_field_basis field_decomp tfield navviz utils)

add_executable(test_threat_map test_threat_map.cpp)
target_link_libraries(test_threat_map field_decomp tfield navviz utils)
