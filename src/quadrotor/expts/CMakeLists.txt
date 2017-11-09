# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
add_executable(expt_cmp_path_01 cmp_path_01.cpp)
target_link_libraries(expt_cmp_path_01 map pr_sim vis ${LCM_LIBRARIES})

add_executable(expt_cmp_path_02 cmp_path_02.cpp)
target_link_libraries(expt_cmp_path_02 map pr_sim vis ${LCM_LIBRARIES})

add_executable(expt_cmp_path_03 cmp_path_03.cpp)
target_link_libraries(expt_cmp_path_03 map pr_sim vis ${LCM_LIBRARIES})

add_executable(expt_pysim_01 pr_pysim_01.cpp)
target_link_libraries(expt_pysim_01 map pr_sim quad_base common ${LCM_LIBRARIES})

add_executable(expt_pysim_02 pr_pysim_02.cpp)
target_link_libraries(expt_pysim_02 map pr_sim quad_base common ${LCM_LIBRARIES})

add_executable(expt_pysim_03 pr_pysim_03.cpp)
target_link_libraries(expt_pysim_03 map pr_sim quad_base common ${LCM_LIBRARIES})

add_executable(expt_pysim_04 pr_pysim_04.cpp)
target_link_libraries(expt_pysim_04 map pr_sim quad_base common ${LCM_LIBRARIES})

add_executable(expt_pysim_05 pr_pysim_05.cpp)
target_link_libraries(expt_pysim_05 map pr_sim quad_base common ${LCM_LIBRARIES})