add_executable(test_odeint test_odeint.cpp)
target_link_libraries(test_odeint odeint)

add_executable(test_robot test_robot.cpp)
target_link_libraries(test_robot simlite model)