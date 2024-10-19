#add_executable(test_evdev test_evdev.cpp)
#target_link_libraries(test_evdev evdev)

add_executable(test_js_event test_js_event.cpp)
target_link_libraries(test_js_event input_hid)

add_executable(test_kb_event test_kb_event.cpp)
target_link_libraries(test_kb_event input_hid)

add_executable(test_both_event test_both_event.cpp)
target_link_libraries(test_both_event input_hid)

add_subdirectory(feature)