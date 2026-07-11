## Allocation tier: malloc-hook counters assert the documented
## zero-heap-allocation contract of the steady-state hot paths —
## Mppi::Plan(), WheeledShield::Filter(), Mekf6/Mekf9::Update(),
## PidController::Update() (the "adopt now" item of
## docs/research/ihmc-open-robotics-software.md §6). The tier is one
## dedicated binary because the malloc interposition is process-global.
##
## Labels:
##   allocation — excluded from PR ctest and the sanitizer lane
##                (-LE "campaign|allocation"; interposition is inactive
##                under ASan/TSan anyway), run nightly. The binary still
##                BUILDS on PRs so it cannot rot.

add_executable(test_allocation alloc_counter.cpp test_allocation.cpp)
target_link_libraries(test_allocation
    estimation mppi pid shield gtest gtest_main)
gtest_discover_tests(test_allocation PROPERTIES LABELS allocation)
