if(BUILD_TESTS)
    add_subdirectory(googletest)
endif()

add_subdirectory(spdlog)
add_subdirectory(rapidcsv)

add_subdirectory(stb)
add_subdirectory(stopwatch)
add_subdirectory(pnm)

add_subdirectory(spatial)
add_subdirectory(quadprog++)


# set(BUILD_DDSPERF OFF CACHE INTERNAL "Build the performance test" FORCE)
# set(BUILD_TESTING OFF CACHE INTERNAL "Build the testing tree" FORCE)
# set(ENABLE_SHM OFF CACHE INTERNAL "Enable shared memory transport" FORCE)
# set(ENABLE_SECURITY OFF CACHE INTERNAL "Enable security" FORCE)
# set(ENABLE_TYPE_DISCOVERY ON CACHE INTERNAL "Enable type discovery" FORCE)
# add_subdirectory(cyclonedds)