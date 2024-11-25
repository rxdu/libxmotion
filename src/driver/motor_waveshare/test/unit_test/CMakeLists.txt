# Google tests
message(STATUS "Build unit tests with Google Test.")

# reference: https://cliutils.gitlab.io/modern-cmake/chapters/testing/googletest.html
mark_as_advanced(
    BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
    gmock_build_tests gtest_build_samples gtest_build_tests
    gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)

# add unit tests
add_executable(waveshare_utests
    utest_ring_buffer.cpp)
target_link_libraries(waveshare_utests PRIVATE gtest gmock gtest_main motor_waveshare)
# get_target_property(PRIVATE_HEADERS imview INCLUDE_DIRECTORIES)
target_include_directories(waveshare_utests PRIVATE ${PRIVATE_HEADERS})

gtest_discover_tests(waveshare_utests)
add_test(NAME gtest_waveshare COMMAND waveshare_utests)