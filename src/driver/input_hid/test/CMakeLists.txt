#add_executable(test_evdev test_evdev.cpp)
#target_link_libraries(test_evdev evdev)

add_executable(test_joystick test_joystick.cpp)
target_link_libraries(test_joystick input_hid)

add_executable(test_keyboard test_keyboard.cpp)
target_link_libraries(test_keyboard input_hid)

add_executable(test_js_event test_js_event.cpp)
target_link_libraries(test_js_event input_hid)

add_executable(test_kb_event test_kb_event.cpp)
target_link_libraries(test_kb_event input_hid)

add_executable(test_both_event test_both_event.cpp)
target_link_libraries(test_both_event input_hid)

add_executable(test_hid_event_listener test_hid_event_listener.cpp)
target_link_libraries(test_hid_event_listener input_hid)