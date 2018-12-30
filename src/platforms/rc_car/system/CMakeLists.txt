## Dependent libraries

# Add libraries
set(COMM_LIB_SRC
    src/nerve_system.cpp
	src/can_messenger.cpp
	src/lcm_messenger.cpp
	# src/lcm_channels.cpp
)
add_library(rccar STATIC ${COMM_LIB_SRC})
target_link_libraries(rccar common socketcan rccar_param rccar_driver logging utils ${LCM_LIBRARIES})
target_include_directories(rccar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# Add executables
#add_executable(car_messenger car_messenger.cpp)
#add_definitions(-DUAVCAN_CPP_VERSION=UAVCAN_CPP11)
#target_link_libraries(car_messenger messenger)
