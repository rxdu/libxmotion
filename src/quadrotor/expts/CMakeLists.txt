# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add executables
add_executable(expt_cmp_path_01 cmp_path_01.cpp)
target_link_libraries(expt_cmp_path_01 map vis path_repair ${OpenCV_LIBS} ${LCM_LIBRARIES})

add_executable(expt_cmp_path_02 cmp_path_02.cpp)
target_link_libraries(expt_cmp_path_02 map vis path_repair ${OpenCV_LIBS} ${LCM_LIBRARIES})