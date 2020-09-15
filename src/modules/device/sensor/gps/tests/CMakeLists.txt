# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
add_executable(gps_demo_simple gps_demo_simple.cpp)
target_link_libraries(gps_demo_simple gps)

add_executable(gps_demo_advanced gps_demo_advanced.cpp)
target_link_libraries(gps_demo_advanced gps)

add_executable(test_gps test_gps.cpp)
target_link_libraries(test_gps gps)

add_executable(test_gps_sensor test_gps_sensor.cpp)
target_link_libraries(test_gps_sensor gps)
