# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(TOOLS_LIB_SRC
	src/i2cbusses.c
    src/i2cdetect.c
	src/i2cdump.c
	src/i2cget.c
	src/i2cset.c 
	src/util.c
)
add_library(tools STATIC ${TOOLS_LIB_SRC})
# target_link_libraries(tools -lpthread)
target_include_directories(tools PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    private src)

add_subdirectory(tests)

