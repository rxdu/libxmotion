# Inspection demos (visual; not ctest-registered — they open windows)
add_executable(demo_mppi_diffdrive demo_mppi_diffdrive.cpp)
target_link_libraries(demo_mppi_diffdrive sim mppi viz)

add_executable(demo_srb_quadruped demo_srb_quadruped.cpp)
target_link_libraries(demo_srb_quadruped sim mppi viz)
