# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(LATTICE_LIB_SRC
)
add_library(lattice ${LATTICE_LIB_SRC})
# target_link_libraries(lattice common)
target_include_directories(lattice INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
