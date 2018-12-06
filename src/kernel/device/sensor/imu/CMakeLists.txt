# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(IMU_LIB_SRC
    src/i2c.cpp
	src/fxos8700cq.cpp
	src/fxas21002c.cpp
)
add_library(imu STATIC ${IMU_LIB_SRC})
# target_link_libraries(imu i2cbus)
target_include_directories(imu PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	private src)

add_subdirectory(tests)

