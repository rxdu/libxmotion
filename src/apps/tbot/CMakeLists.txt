add_executable(app_tbot_widget
        src/app_tbot_widget.cpp
        src/tbot_widget.cpp
        src/control_panel.cpp
        src/plot_panel.cpp
        src/canbus_panel.cpp)
target_link_libraries(app_tbot_widget imview async_port)
target_include_directories(app_tbot_widget PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src)