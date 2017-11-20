# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_create_field test_create_field.cpp)
target_link_libraries(test_create_field field)

add_executable(test_road_field test_road_field.cpp)
target_link_libraries(test_road_field field ${LCM_LIBRARIES}) 

add_executable(test_vehicle_field test_vehicle_field.cpp)
target_link_libraries(test_vehicle_field field ${LCM_LIBRARIES}) 