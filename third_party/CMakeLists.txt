if (BUILD_TESTS)
  if (TARGET gtest)
    message(STATUS "GoogleTest already found, project will use the existing target")
  else ()
    add_subdirectory(googletest)
  endif ()
endif ()

add_subdirectory(spdlog)
add_subdirectory(rapidcsv)

add_subdirectory(stb)
add_subdirectory(pnm)
add_subdirectory(stopwatch)

add_subdirectory(spatial)
add_subdirectory(quadprog++)
