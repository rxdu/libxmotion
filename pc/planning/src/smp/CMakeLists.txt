# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
#set(SMP_LIB_SRC
#    square_grid.cpp
#)
#add_library(smp STATIC ${SMP_LIB_SRC})
#target_link_libraries(smp ${OpenCV_LIBS})

# Add executables
add_executable(dubins_rrtstar standalone_rrtstar_dubins_car.cpp)
target_link_libraries(dubins_rrtstar ${SMP_KARAMAN_LIB})

#add_executable(test_ompl test_ompl.cpp)
#target_link_libraries(test_ompl ompl)

#add_executable(test_smp test_smp.cpp)
#target_link_libraries(test_smp ompl)
