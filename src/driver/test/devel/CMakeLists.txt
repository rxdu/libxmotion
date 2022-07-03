add_executable(test_hipnuc test_hipnuc.cpp)
target_link_libraries(test_hipnuc driver)

add_executable(test_vesc_driver test_vesc_driver.cpp)
target_link_libraries(test_vesc_driver driver)

