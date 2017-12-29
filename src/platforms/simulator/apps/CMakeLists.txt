# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Apps based on AscTec Hummingbird
add_executable(quadsim_hbird_demo quadsim_hbird_demo.cpp)
target_link_libraries(quadsim_hbird_demo quadsim)

#add_executable(rc_car_sim_demo rc_car_sim_demo.cpp)
#target_link_libraries(rc_car_sim_demo rccarsim car_ctrl)

#add_executable(rc_car_sim_control rc_car_sim_control.cpp)
#target_link_libraries(rc_car_sim_control rccarsim car_ctrl)

# add_executable(quadsim_path_repair quadsim_path_repair.cpp)
# target_link_libraries(quadsim_path_repair quadsim path_repair)

# add_executable(quadsim_path_repair_test quadsim_path_repair_test.cpp)
# target_link_libraries(quadsim_path_repair_test quadsim path_repair)

add_executable(quadsim_hbird_pos_step quadsim_hbird_pos_step.cpp)
target_link_libraries(quadsim_hbird_pos_step quadsim)

# Apps based on 3DR Solo
add_executable(quadsim_solo_demo quadsim_solo_demo.cpp)
target_link_libraries(quadsim_solo_demo quadsolosim)
