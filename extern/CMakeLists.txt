if(BUILD_TESTS)
    add_subdirectory(googletest)
endif()

add_subdirectory(spdlog)
add_subdirectory(rapidcsv)

add_library(stopwatch INTERFACE)
target_include_directories(stopwatch INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/stopwatch>
    $<INSTALL_INTERFACE:include>)

add_subdirectory(spatial)

# set(BUILD_DDSPERF OFF CACHE INTERNAL "Build the performance test" FORCE)
# set(BUILD_TESTING OFF CACHE INTERNAL "Build the testing tree" FORCE)
# set(ENABLE_SHM OFF CACHE INTERNAL "Enable shared memory transport" FORCE)
# set(ENABLE_SECURITY OFF CACHE INTERNAL "Enable security" FORCE)
# set(ENABLE_TYPE_DISCOVERY ON CACHE INTERNAL "Enable type discovery" FORCE)
# add_subdirectory(cyclonedds)