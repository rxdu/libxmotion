## Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

## Find LCM
find_package(LCM REQUIRED)

if(LCM_NOT_FOUND)
  message(STATUS "LCM is not found. Inter-process communication depends on LCM.")
endif(LCM_NOT_FOUND)

# Include files for LCM
set(LCM_GENERATED_HEADER_PATH ${CMAKE_BINARY_DIR}/include)
include_directories(${LCM_GENERATED_HEADER_PATH})

# Add libraries
set(COMM_LIB_SRC
  src/nerve_system.cpp
	src/can_messenger.cpp
	src/lcm_messenger.cpp
	src/lcm_channels.cpp
)
add_library(rccar STATIC ${COMM_LIB_SRC})
target_link_libraries(rccar common socketcan rccar_driver utilities ${LCM_LIBRARIES})
target_include_directories(rccar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# Add executables
#add_executable(car_messenger car_messenger.cpp)
#add_definitions(-DUAVCAN_CPP_VERSION=UAVCAN_CPP11)
#target_link_libraries(car_messenger messenger)
