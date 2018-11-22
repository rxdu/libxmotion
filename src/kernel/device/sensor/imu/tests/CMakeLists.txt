# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
add_executable(test_fxos8700 test_fxos8700cq.cpp)
target_link_libraries(test_fxos8700 sensors utils)

add_executable(test_fxas21002 test_fxas21002c.cpp)
target_link_libraries(test_fxas21002 sensors utils)

add_executable(test_imu test_imu.cpp)
target_link_libraries(test_imu sensors utils)
