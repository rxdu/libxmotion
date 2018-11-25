cmake_minimum_required(VERSION 3.0.0)
project(librav_libs)

## Put all binary files into /bin and libraries into /lib
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

## Add sub source directories
add_subdirectory(asio)
add_subdirectory(googletest)
add_subdirectory(spdlog)
add_subdirectory(ascent)
add_subdirectory(liblanelet)
add_subdirectory(gurobi_cpp)
add_subdirectory(osqp)
add_subdirectory(eigenfileio)
add_subdirectory(imgui)
# add_subdirectory(kindr)
# add_subdirectory(polynomial)
add_subdirectory(dubins_curves)
add_subdirectory(nema_tode)
