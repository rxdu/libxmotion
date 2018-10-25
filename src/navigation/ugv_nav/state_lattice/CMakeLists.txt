# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(LATTICE_LIB_SRC
  src/point_kinematics.cpp
  src/primitive_generator.cpp
)
add_library(lattice STATIC ${LATTICE_LIB_SRC})
target_link_libraries(lattice graph geometry model ascent utilities)
target_include_directories(lattice PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
