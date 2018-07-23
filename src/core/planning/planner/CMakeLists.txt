# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(GEOMETRY_LIB_SRC
  # builders
  src/graph_builder.cpp
)
add_library(planner STATIC ${GEOMETRY_LIB_SRC})
target_link_libraries(planner graph decomp)
target_include_directories(planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

