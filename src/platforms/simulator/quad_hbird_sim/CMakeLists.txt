# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(QUADSIM_LIB_SRC
    src/quad_hbird_sim_client.cpp
    src/quad_hbird_sim_control.cpp
)
add_library(quad_sim STATIC ${QUADSIM_LIB_SRC})
target_link_libraries(quad_sim sim_interface common quad_ctrl quad_sys datalink)
target_include_directories(quad_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# Add executables
add_subdirectory(apps)
