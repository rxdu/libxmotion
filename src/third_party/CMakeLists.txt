## Add sub source directories
add_subdirectory(asio)
add_subdirectory(spdlog)
add_subdirectory(ascent)
add_subdirectory(liblanelet)
add_subdirectory(gurobi_cpp)
add_subdirectory(osqp)
add_subdirectory(eigenfileio)
add_subdirectory(spatial)
add_subdirectory(dubins_curves)
# add_subdirectory(kindr)
# add_subdirectory(polynomial)
# visualization
add_subdirectory(glad)
add_subdirectory(imgui)
# add_subdirectory(drawstuff)

if(BUILD_TESTS)
    # Google tests
    message(STATUS "Build unit tests with Google Test.")
    add_subdirectory(googletest)

    # reference: https://cliutils.gitlab.io/modern-cmake/chapters/testing/googletest.html
    mark_as_advanced(
        BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
        gmock_build_tests gtest_build_samples gtest_build_tests
        gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
    )
endif()