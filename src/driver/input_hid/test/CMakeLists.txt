#add_executable(test_evdev test_evdev.cpp)
#target_link_libraries(test_evdev evdev)

add_executable(test_joystick test_joystick.cpp)
target_link_libraries(test_joystick input_hid)

add_executable(test_keyboard test_keyboard.cpp)
target_link_libraries(test_keyboard input_hid)

add_executable(test_hid_event_poll test_hid_event_poll.cpp)
target_link_libraries(test_hid_event_poll input_hid)