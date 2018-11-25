# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(GPS_LIB_SRC
    src/gps_receiver.cpp
)
add_library(gps STATIC ${GPS_LIB_SRC})
target_link_libraries(gps datalink asyncserial nemaparse)
target_include_directories(gps PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	private src)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

