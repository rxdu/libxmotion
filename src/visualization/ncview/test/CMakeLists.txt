add_executable(test_ncurses test_ncurses.cpp)
target_link_libraries(test_ncurses ncview)

add_executable(test_terminal test_terminal.cpp)
target_link_libraries(test_terminal ncview)

#add_executable(test_ncview_memory test_ncview_memory.cpp)
#target_link_libraries(test_ncview_memory ncview)