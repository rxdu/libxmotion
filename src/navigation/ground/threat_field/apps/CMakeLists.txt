# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(app_compute_transition app_compute_transition.cpp)
target_link_libraries(app_compute_transition tfield ugvnav_viz utilities)