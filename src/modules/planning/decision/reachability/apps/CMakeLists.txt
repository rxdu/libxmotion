# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(gen_tstate_transition gen_tstate_transition.cpp)
target_link_libraries(gen_tstate_transition reachability utils)