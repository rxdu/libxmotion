add_definitions(-std=gnu99)

include_directories(${PROJECT_SOURCE_DIR}/src/)
add_executable(bot-lcmgl-viewer
    main.c
    udp_util.c
    view_menu.c
    )
target_link_libraries(bot-lcmgl-viewer bot2-lcmgl-renderer)
pods_use_pkg_config_packages(bot-lcmgl-viewer bot2-vis glib-2.0 gthread-2.0)


pods_install_executables(bot-lcmgl-viewer)
