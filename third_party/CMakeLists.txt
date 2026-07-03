if (BUILD_TESTS)
  if (TARGET gtest)
    message(STATUS "GoogleTest already found, project will use the existing target")
  else ()
    add_subdirectory(googletest)
  endif ()
endif ()


if (NOT USE_SYS_SPDLOG)
  add_subdirectory(spdlog)
endif ()

add_subdirectory(rapidcsv)

add_subdirectory(stb)
add_subdirectory(pnm)
add_subdirectory(stopwatch)
# rpi_sbus is now provided by xmDriver (used by its input_sbus driver). Adding it
# here too would create a duplicate target in the superbuild, so it is dropped
# from xmNavigation's own third_party.

add_subdirectory(spatial)
add_subdirectory(quadprog++)
