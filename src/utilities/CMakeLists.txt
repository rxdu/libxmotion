# Add libraries
add_subdirectory(file_io)
# add_subdirectory(image_io)
add_subdirectory(logging)
add_subdirectory(time)

add_library(utilities INTERFACE)
target_link_libraries(utilities INTERFACE file_io logging time)