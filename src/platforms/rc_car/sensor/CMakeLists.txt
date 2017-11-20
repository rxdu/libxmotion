## Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add libraries
set(SENSOR_LIB_SRC
	imu_filter.cpp
)
add_library(sensor STATIC ${SENSOR_LIB_SRC})
target_link_libraries(sensor common)

# Add executables
#add_executable(car_messenger car_messenger.cpp)
#add_definitions(-DUAVCAN_CPP_VERSION=UAVCAN_CPP11)
#target_link_libraries(car_messenger comm)
