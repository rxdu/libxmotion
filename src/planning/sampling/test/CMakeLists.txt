# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_rvspace test_rvspace.cpp)
target_link_libraries(test_rvspace sampling interface)

add_executable(test_rng test_rng.cpp)
target_link_libraries(test_rng sampling)

add_executable(test_rrt test_rrt.cpp)
target_link_libraries(test_rrt sampling interface)

add_executable(test_kdtree test_kdtree.cpp)
target_link_libraries(test_kdtree sampling interface)

add_executable(test_rrg test_rrg.cpp)
target_link_libraries(test_rrg sampling interface)

add_executable(test_rrts test_rrts.cpp)
target_link_libraries(test_rrts sampling interface)

add_executable(test_rrt_draw test_rrt_draw.cpp)
target_link_libraries(test_rrt_draw sampling)
