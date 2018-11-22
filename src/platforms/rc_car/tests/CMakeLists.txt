# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
# add_executable(test_imu_calib test_imu_calib.cpp)
# target_link_libraries(test_imu_calib sensor utility)

add_executable(test_can_cmd test_can_cmd.cpp)
target_link_libraries(test_can_cmd rccar)

add_executable(test_imu_driver test_imu_driver.cpp)
target_link_libraries(test_imu_driver rccar_driver utility)

# add_executable(test_inc_path test_create_field.cpp)
# target_link_libraries(test_inc_path field)
