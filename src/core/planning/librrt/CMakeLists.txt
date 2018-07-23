# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(RRT_LIB_SRC
)
add_library(rrt INTERFACE)
# target_link_libraries(rrt common)
target_sources(rrt INTERFACE ${RRT_LIB_SRC})
target_include_directories(rrt INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_subdirectory(tests)

