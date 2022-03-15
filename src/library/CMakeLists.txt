add_subdirectory(spdlog)
add_subdirectory(rapidcsv)

if(BUILD_TESTS)
    add_subdirectory(googletest)
endif()
