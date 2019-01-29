# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)

# Dependency libraries
# find_package(FLTK REQUIRED)
# include_directories(${FLTK_INCLUDE_DIR})

## tests
add_executable(test_fltk test_fltk.cpp)
target_link_libraries(test_fltk ${FLTK_LIBRARIES})