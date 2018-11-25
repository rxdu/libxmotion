# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
add_executable(read_whoami read_whoami.cpp)
target_link_libraries(read_whoami imu)

add_executable(test_i2cwrite test_i2cwrite.cpp)
target_link_libraries(test_i2cwrite imu)

add_executable(test_fxos8700 test_fxos8700cq.cpp)
target_link_libraries(test_fxos8700 imu utils)

add_executable(test_fxas21002 test_fxas21002c.cpp)
target_link_libraries(test_fxas21002 imu utils)

add_executable(test_imu test_imu.cpp)
target_link_libraries(test_imu imu utils)
