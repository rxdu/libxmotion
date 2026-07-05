# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(LATTICE_LIB_SRC
  src/point_kinematics.cpp
  src/motion_primitive.cpp
  src/lookup_table.cpp
  src/primitive_generator.cpp
  src/state_lattice.cpp
)
if (ENABLE_VISUALIZATION)
  list(APPEND LATTICE_LIB_SRC src/lattice_draw.cpp src/lattice_viz.cpp)
endif ()
add_library(lattice STATIC ${LATTICE_LIB_SRC})
target_link_libraries(lattice geometry)
if (ENABLE_VISUALIZATION)
  target_compile_definitions(lattice PUBLIC ENABLE_VISUAL)
  target_link_libraries(lattice cvdraw)
endif ()
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
