## Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add libraries
set(RCCAR_DRIVER_LIB_SRC
	src/imu_neo.cpp
)
add_library(rccar_driver STATIC ${RCCAR_DRIVER_LIB_SRC})
target_link_libraries(rccar_driver common sensors utilities ${LCM_LIBRARIES})
target_include_directories(rccar_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# Add executables
#add_executable(car_messenger car_messenger.cpp)
#add_definitions(-DUAVCAN_CPP_VERSION=UAVCAN_CPP11)
#target_link_libraries(car_messenger messenger)

add_subdirectory(tests)