# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
add_executable(expt_static_field static_field.cpp)
target_link_libraries(expt_static_field field traffic utilities fastplot ${LCM_LIBRARIES})

add_executable(static_field_animation static_field_animation.cpp)
target_link_libraries(static_field_animation field traffic utilities fastplot ${LCM_LIBRARIES})
