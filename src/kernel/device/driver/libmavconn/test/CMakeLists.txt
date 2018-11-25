# Add executables
add_executable(test_interface test_interface.cpp)
target_link_libraries(test_interface mavconn)

add_executable(test_aserial_if test_aserial_if.cpp)
target_link_libraries(test_aserial_if mavconn)
