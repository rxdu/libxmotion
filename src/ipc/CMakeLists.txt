find_package(lcm REQUIRED)
include(${LCM_USE_FILE})

lcm_wrap_types(
        C_EXPORT ipc_lcmtypes
        C_SOURCES c_sources
        C_HEADERS c_headers
        CPP_HEADERS cpp_headers
        lcmtypes/example_t.lcm)

lcm_add_library(ipc_lcmtypes-cpp CPP ${cpp_headers})
target_include_directories(ipc_lcmtypes-cpp INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()

###########################################################

#lcm_install_headers(DESTINATION include
#        ${CMAKE_CURRENT_BINARY_DIR}/ipc_lcmtypes_export.h
#        ${c_install_headers}
#        ${cpp_install_headers})
#
#if (PYTHONINTERP_FOUND)
#    lcm_install_python(${python_install_sources})
#endif ()
#
#install(TARGETS ipc_lcmtypes-cpp
#        EXPORT ${PROJECT_NAME}Targets
#        RUNTIME DESTINATION bin
#        LIBRARY DESTINATION lib${LIB_SUFFIX}
#        ARCHIVE DESTINATION lib${LIB_SUFFIX}
#        INCLUDES DESTINATION include)
