add_executable(test_ddsm210 test_ddsm210.cpp)
target_link_libraries(test_ddsm210 motor_waveshare)

add_executable(test_servo_broadcast ./examples/SMS_STS/Broadcast/Broadcast.cpp)
target_link_libraries(test_servo_broadcast motor_waveshare)
target_include_directories(test_servo_broadcast PRIVATE ../src/servo)