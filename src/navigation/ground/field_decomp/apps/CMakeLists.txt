# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(app_calc_transition app_calc_transition.cpp)
target_link_libraries(app_calc_transition tfield navviz utils)

add_executable(app_calc_interval_trans app_calc_interval_trans.cpp)
target_link_libraries(app_calc_interval_trans tfield navviz utils)