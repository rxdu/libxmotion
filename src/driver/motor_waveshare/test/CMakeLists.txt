add_executable(test_ddsm210 test_ddsm210.cpp)
target_link_libraries(test_ddsm210 motor_waveshare)

add_executable(test_ddsm210_array test_ddsm210_array.cpp)
target_link_libraries(test_ddsm210_array motor_waveshare)

add_executable(test_servo_ping ./examples/SMS_STS/Ping/Ping.cpp)
target_link_libraries(test_servo_ping motor_waveshare)
target_include_directories(test_servo_ping PRIVATE ../src/servo)

add_executable(test_servo_write_pos ./examples/SMS_STS/WritePos/WritePos.cpp)
target_link_libraries(test_servo_write_pos motor_waveshare)
target_include_directories(test_servo_write_pos PRIVATE ../src/servo)