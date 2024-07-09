# Add executables
add_executable(test_logging test_logging.cpp)
target_link_libraries(test_logging logging)

add_executable(test_xlogger test_xlogger.cpp)
target_link_libraries(test_xlogger logging)