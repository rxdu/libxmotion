# Inspection demos (visual; not ctest-registered — they open windows)
add_executable(demo_mppi_diffdrive demo_mppi_diffdrive.cpp)
target_link_libraries(demo_mppi_diffdrive sim mppi viz)

add_executable(demo_srb_quadruped demo_srb_quadruped.cpp)
target_link_libraries(demo_srb_quadruped sim mppi viz)

# Interactive MPPI tuner: quickviz viewer shell (Box layout), cairo world
# view with the alpha-blended candidate fan, live ESS/cost plots,
# pause/step and lambda/sigma controls
add_executable(tuner_mppi_diffdrive tuner_mppi_diffdrive.cpp)
target_link_libraries(tuner_mppi_diffdrive sim mppi viz viewer plot)
