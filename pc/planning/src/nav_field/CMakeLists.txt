# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(NAV_FIELD_LIB_SRC
    shortcut_eval.cpp
)
add_library(nav_field STATIC ${NAV_FIELD_LIB_SRC})
target_link_libraries(nav_field geometry)# ${OpenCV_LIBS} ${LCM_LIBRARIES})

# Add executables
#add_executable(graph_combine_exp tests/graph_combine_exp.cpp)
#target_link_libraries(graph_combine_exp map local3d utils ${Octomap_LIBS} ${LCM_LIBRARIES})

add_executable(test_calc tests/test_calc.cpp)
target_link_libraries(test_calc nav_field map vis ${OpenCV_LIBS})

add_executable(test_navfield tests/test_navfield.cpp)
target_link_libraries(test_navfield map vis ${OpenCV_LIBS})

add_executable(test_navfield2 tests/test_navfield2.cpp)
target_link_libraries(test_navfield2 map vis nav_field ${OpenCV_LIBS})
