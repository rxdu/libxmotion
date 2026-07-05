# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
add_library(lattice STATIC
  src/point_kinematics.cpp
  src/motion_primitive.cpp
  src/lookup_table.cpp
  src/primitive_generator.cpp
  src/state_lattice.cpp)
target_link_libraries(lattice geometry)
target_include_directories(lattice PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# apps/ excluded: gen_lookup_table depends on the retired traffic_map
# module; re-enable with the road-network revival.

install(TARGETS lattice
    EXPORT xmNavigationTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
