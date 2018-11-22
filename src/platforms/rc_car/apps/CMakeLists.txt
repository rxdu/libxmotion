# Add executables
# add_executable(car_control car_control.cpp)
# add_definitions(-DUAVCAN_CPP_VERSION=UAVCAN_CPP11)
# target_link_libraries(car_control messenger sensor utility)

add_executable(rccar_main rccar_main.cpp)
target_link_libraries(rccar_main rccar)
