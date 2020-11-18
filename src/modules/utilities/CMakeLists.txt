# Add libraries
add_subdirectory(file_io)
add_subdirectory(image_io)
add_subdirectory(logging)
add_subdirectory(third_party)

add_library(utils INTERFACE)
target_link_libraries(utils INTERFACE file_io image_io logging stopwatch csv_parser)