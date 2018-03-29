# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(DECOMP_LIB_SRC
  src/square_grid.cpp
)
add_library(decomp STATIC ${DECOMP_LIB_SRC})
target_link_libraries(decomp common)
target_include_directories(decomp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

add_subdirectory(tests)

