# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
add_executable(test_imu_calib test_imu_calib.cpp)
target_link_libraries(test_imu_calib sensor utility)

add_executable(test_inc_path test_create_field.cpp)
target_link_libraries(test_inc_path field)
