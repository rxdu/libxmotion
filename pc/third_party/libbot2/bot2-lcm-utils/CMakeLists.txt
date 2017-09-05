cmake_minimum_required(VERSION 2.6.0)

include(cmake/pods.cmake)

add_subdirectory(src/logfilter)
add_subdirectory(src/logsplice)
add_subdirectory(src/who)
add_subdirectory(src/tunnel)
add_subdirectory(python)


# bot-spy
set(script_name bot-spy)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/${script_name}.sh.in
  ${CMAKE_CURRENT_BINARY_DIR}/${script_name}
  @ONLY)

# install it...
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${script_name} DESTINATION bin)
