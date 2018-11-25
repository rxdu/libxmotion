## Dependent libraries

# Add libraries
set(CARPARAM_LIB_SRC
	src/lcm_channels.cpp
)
add_library(rccar_param STATIC ${CARPARAM_LIB_SRC})
target_link_libraries(rccar_param common utils)
target_include_directories(rccar_param PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# Add executables
#add_executable(car_messenger car_messenger.cpp)
#add_definitions(-DUAVCAN_CPP_VERSION=UAVCAN_CPP11)
#target_link_libraries(car_messenger messenger)
