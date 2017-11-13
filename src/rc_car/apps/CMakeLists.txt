## Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add executables
add_executable(car_control car_control.cpp)
add_definitions(-DUAVCAN_CPP_VERSION=UAVCAN_CPP11)
target_link_libraries(car_control comm sensor utility)
