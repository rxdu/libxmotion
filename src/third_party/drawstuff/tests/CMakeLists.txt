# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# tests
add_executable(test_draw_stuff test_draw_stuff.cpp)
target_link_libraries(test_draw_stuff drawstuff)
