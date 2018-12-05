# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_rccar_odesim test_rccar_odesim.cpp)
target_link_libraries(test_rccar_odesim rccar_ctrl)