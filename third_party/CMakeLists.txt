if(BUILD_TESTS)
    add_subdirectory(googletest)
endif()

add_subdirectory(spdlog)
add_subdirectory(rapidcsv)

add_subdirectory(stb)
add_subdirectory(pnm)
add_subdirectory(stopwatch)

add_subdirectory(spatial)
add_subdirectory(quadprog++)
