add_executable(test_hipnuc test_hipnuc.cpp)
target_link_libraries(test_hipnuc sensor_imu)

add_executable(test_imu_plot test_imu_plot.cpp)
target_link_libraries(test_imu_plot sensor_imu imview)

