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
	can_messenger.cpp
	lcm_messenger.cpp
	lcm_channels.cpp
)
add_library(comm STATIC ${COMM_LIB_SRC})
add_definitions(-DUAVCAN_CPP_VERSION=UAVCAN_CPP11)
target_link_libraries(comm uavcan ${LCM_LIBRARIES})
target_include_directories(comm PUBLIC ${CMAKE_BINARY_DIR}/include/uavcantypes)

# Add executables
#add_executable(car_messenger car_messenger.cpp)
#add_definitions(-DUAVCAN_CPP_VERSION=UAVCAN_CPP11)
#target_link_libraries(car_messenger comm)
