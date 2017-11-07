# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
set(PATH_REPAIR_COMMON_SRC
    shortcut_eval.cpp
    sg_graph_planner.cpp
)

if (GUROBI_FOUND)
    set(PATH_REPAIR_LIB_SRC
        quad_path_repair.cpp    
        geo_mark_graph.cpp    
    )
    add_library(path_repair STATIC ${PATH_REPAIR_LIB_SRC} ${PATH_REPAIR_COMMON_SRC})
    target_link_libraries(path_repair map quad_base polyopt common vis ${OpenCV_LIBS} ${LCM_LIBRARIES})
endif()

set(PATH_REPAIR_SIM_SRC
    sim/sim_path_repair.cpp
    sim/virtual_quadrotor.cpp
    sim/sim_depth_sensor.cpp
)
add_library(pr_sim STATIC ${PATH_REPAIR_SIM_SRC} ${PATH_REPAIR_COMMON_SRC})
target_link_libraries(pr_sim utility ${LCM_LIBRARIES} ${OpenCV_LIBS})

# Add executables

# Tests
add_subdirectory(tests)