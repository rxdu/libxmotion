# Dependency libraries
# find_package(LIB_NAME REQUIRED)

# Add libraries
set(LOGGING_LIB_SRC
    src/csv_logger.cpp
    src/ctrl_logger.cpp
	src/event_logger.cpp
)
add_library(logging STATIC ${LOGGING_LIB_SRC})
if(ENABLE_LOGGING)
target_compile_definitions(logging PUBLIC -DENABLE_LOGGING)
endif(ENABLE_LOGGING)
target_link_libraries(logging spdlog)
target_include_directories(logging PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()