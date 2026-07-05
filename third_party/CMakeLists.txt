if (BUILD_TESTS)
  if (TARGET gtest)
    message(STATUS "GoogleTest already found, project will use the existing target")
  else ()
    add_subdirectory(googletest)
  endif ()
endif ()

add_subdirectory(rapidcsv)

add_subdirectory(stb)
add_subdirectory(pnm)
add_subdirectory(stopwatch)

add_subdirectory(spatial)
add_subdirectory(quadprog++)

# Visualization engine (vendored; used by viz-gated targets only)
if (ENABLE_VISUALIZATION)
  option(QUICKVIZ_DEV_MODE "Build quickviz in dev mode" ON)
  option(BUILD_QUICKVIZ_APP "Build quickviz application" ON)
  add_subdirectory(quickviz)
endif ()
