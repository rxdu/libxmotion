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

# xmViewer (quickviz): a FAMILY development-support component, not
# third-party code — consumed by the visualization-gated modules and demos
# only, never part of the deployment closure (ADR 0005 dependency classes).
if (ENABLE_VISUALIZATION)
  # library modules only — upstream samples, apps, and tests are not part
  # of this build (xmViewer has its own CI)
  option(QUICKVIZ_DEV_MODE "Build quickviz in dev mode" OFF)
  option(BUILD_QUICKVIZ_APP "Build quickviz application" OFF)
  set(_xm_build_testing ${BUILD_TESTING})
  set(BUILD_TESTING OFF)
  # honor normal variables over option() in the subtree (CMP0077)
  set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
  add_subdirectory(quickviz)
  set(BUILD_TESTING ${_xm_build_testing})
  unset(_xm_build_testing)
endif ()
