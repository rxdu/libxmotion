# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

set(CTRL_UTILS_LOG_SRC )
if(ENABLE_LOGGING)
	set(CTRL_UTILS_LOG_SRC
	    logging/logging_helper.cpp
	    )
endif(ENABLE_LOGGING)

# Add libraries
set(CTRL_UTILS_LIB_SRC
    # param
)
if(ENABLE_LOGGING)
add_library(ctrl_utils STATIC ${CTRL_UTILS_LIB_SRC} ${CTRL_UTILS_LOG_SRC})
target_link_libraries(ctrl_utils g3logger)
endif(ENABLE_LOGGING)


# Add executables
#add_executable(test_logging logging/tests/test_logging.cpp)
#target_link_libraries(test_logging ctrl_utils -pthread)
