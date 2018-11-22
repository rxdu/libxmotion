# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(I2CBUS_LIB_SRC
	src/i2c.cpp
)
add_library(i2cbus STATIC ${I2CBUS_LIB_SRC})
# target_link_libraries(i2cbus -lpthread)
target_include_directories(i2cbus PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	private src)

add_subdirectory(tests)

