cmake_minimum_required(VERSION 2.8.3)
project(libmavconn)

## Declare a cpp library
# add_library(mavconn
#   src/interface.cpp
#   src/serial.cpp
# )
# target_link_libraries(mavconn
#   ${Boost_LIBRARIES}
#   ${console_bridge_LIBRARIES}
# )

# Add libraries
set(ASYNC_SERIAL_LIB_SRC
    src/interface.cpp
    src/serial.cpp
    src/async_serial.cpp
)
add_library(mavconn STATIC ${ASYNC_SERIAL_LIB_SRC})
target_link_libraries(mavconn asio pthread)
target_include_directories(mavconn PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

add_subdirectory(test)