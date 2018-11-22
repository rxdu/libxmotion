# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(SENSORS_LIB_SRC
	src/fxos8700cq.cpp
	src/fxas21002c.cpp
)
add_library(sensors STATIC ${SENSORS_LIB_SRC})
target_link_libraries(sensors i2cbus)
target_include_directories(sensors PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	private src)

add_subdirectory(tests)

