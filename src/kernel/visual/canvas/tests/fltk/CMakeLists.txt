# Dependency libraries
# find_package(FLTK REQUIRED)

## tests
add_executable(test_fltk test_fltk.cpp)
target_link_libraries(test_fltk ${FLTK_LIBRARIES})