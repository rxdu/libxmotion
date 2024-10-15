# Google tests
message(STATUS "Build unit tests with Google Test.")

# reference: https://cliutils.gitlab.io/modern-cmake/chapters/testing/googletest.html
mark_as_advanced(
    BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
    gmock_build_tests gtest_build_samples gtest_build_tests
    gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)

# add unit tests
add_executable(utest_async_port
    utest_ring_buffer.cpp)
target_link_libraries(utest_async_port PRIVATE gtest gmock gtest_main async_port)
# get_target_property(PRIVATE_HEADERS imview INCLUDE_DIRECTORIES)
target_include_directories(utest_async_port PRIVATE ${PRIVATE_HEADERS})

gtest_discover_tests(utest_async_port)
add_test(NAME gtest_async_port COMMAND utest_async_port)