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

add_executable(expt_pysim_01 pr_pysim_01.cpp)
target_link_libraries(expt_pysim_01 map path_repair pr_sim quad_base common ${OpenCV_LIBS} ${LCM_LIBRARIES})

add_executable(expt_pysim_02 pr_pysim_02.cpp)
target_link_libraries(expt_pysim_02 map path_repair pr_sim quad_base common ${OpenCV_LIBS} ${LCM_LIBRARIES})

add_executable(expt_pysim_03 pr_pysim_03.cpp)
target_link_libraries(expt_pysim_03 map path_repair pr_sim quad_base common ${OpenCV_LIBS} ${LCM_LIBRARIES})

add_executable(expt_pysim_04 pr_pysim_04.cpp)
target_link_libraries(expt_pysim_04 map path_repair pr_sim quad_base common ${OpenCV_LIBS} ${LCM_LIBRARIES})

add_executable(expt_pysim_05 pr_pysim_05.cpp)
target_link_libraries(expt_pysim_05 map path_repair pr_sim quad_base common ${OpenCV_LIBS} ${LCM_LIBRARIES})