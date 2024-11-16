add_executable(app_quadruped_locomotion app_quadruped_locomotion.cpp)
target_link_libraries(app_quadruped_locomotion PRIVATE quadruped logging)

if (ENABLE_VISUALIZATION)
  add_executable(app_unitree_monitor app_unitree_monitor.cpp)
  target_link_libraries(app_unitree_monitor PRIVATE quadruped logging imview)
endif ()