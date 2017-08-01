cmake_minimum_required(VERSION 2.8.11)
project(ave_third_party)

## Put all binary files into /bin and libraries into /lib
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

## Add sub source directories
#add_subdirectory(eigen)
#add_subdirectory(g3log)
add_subdirectory(octomap-1.8.0)
add_subdirectory(googletest)
