# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(RCCARSIM_LIB_SRC
    src/rc_tamiya_sim_client.cpp
    src/rc_tamiya_sim_control.cpp
)
add_library(rccar_sim STATIC ${RCCARSIM_LIB_SRC})
target_link_libraries(rccar_sim sim_interface common utils datalink)
target_include_directories(rccar_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# Add executables
#add_executable(quadsim_demo quadsim_demo.cpp)
#target_link_libraries(quadsim_demo quadsim quad_ctrl)
