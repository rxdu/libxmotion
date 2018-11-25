# Add executables
add_executable(rc_tamiya_sim_demo rc_tamiya_sim_demo.cpp)
target_link_libraries(rc_tamiya_sim_demo rccar_sim)

#add_executable(quadsim_demo quadsim_demo.cpp)
#target_link_libraries(quadsim_demo quadsim quad_ctrl)
