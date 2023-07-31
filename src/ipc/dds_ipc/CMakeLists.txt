#find_package(lcm REQUIRED)
#include(${LCM_USE_FILE})
#
#lcm_wrap_types(
#        C_EXPORT ipc_lcmtypes
#        C_SOURCES c_sources
#        C_HEADERS c_headers
#        CPP_HEADERS cpp_headers
#        lcmtypes/example_t.lcm)
#
#lcm_add_library(ipc_lcmtypes-cpp CPP ${cpp_headers})
#target_include_directories(ipc_lcmtypes-cpp INTERFACE
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()
