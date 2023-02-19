# find dependency
set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)

# add library
add_library(ncview
        src/nc_viewer.cpp
        src/nc_subwindow.cpp
        src/nc_container.cpp
        src/nc_hbox.cpp
        src/nc_vbox.cpp
        src/nc_text.cpp)
target_link_libraries(ncview PUBLIC ${CURSES_LIBRARIES})
target_include_directories(ncview PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src)

if (BUILD_TESTS)
    add_subdirectory(test)
endif ()
