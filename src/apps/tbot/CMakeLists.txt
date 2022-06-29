add_executable(app_tbot_widget
        src/app_tbot_widget.cpp
        src/tbot_widget.cpp
        src/control_panel.cpp
        src/plot_panel.cpp
        src/data_panel.cpp
        src/canbus_panel.cpp
        src/messenger.cpp
        src/speed_controller.cpp)
target_link_libraries(app_tbot_widget interface imview driver pid)
target_include_directories(app_tbot_widget PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src)

add_executable(tbot_speed_control src/tbot_speed_control.cpp)
target_link_libraries(tbot_speed_control interface pid driver)