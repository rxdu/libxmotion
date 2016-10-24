# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

set(CTRL_UTILS_LOG_SRC )
if(ENABLE_G3LOG)
	set(CTRL_UTILS_LOG_SRC
	    logging/logging_helper.cpp
	    )
endif(ENABLE_G3LOG)

# Add libraries
set(CTRL_UTILS_LIB_SRC
    # vis
    #vis/quadvis_data_transmitter.cpp
)
add_library(ctrl_utils STATIC ${CTRL_UTILS_LIB_SRC} ${CTRL_UTILS_LOG_SRC})
#add_definitions(-DENABLE_G3LOG)
target_link_libraries(ctrl_utils g3logger ${LCM_LIBRARIES})

# Add executables
#add_executable(test_logging logging/tests/test_logging.cpp)
#target_link_libraries(test_logging ctrl_utils -pthread)
