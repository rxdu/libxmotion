# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(RCCARSIM_LIB_SRC
    src/rc_tamiya_sim_client.cpp
    src/rc_tamiya_sim_coordinator.cpp
)
add_library(rccar_sim STATIC ${RCCARSIM_LIB_SRC})
target_link_libraries(rccar_sim sim_interface common datalink rccar_shared logging utils)
target_include_directories(rccar_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# Add executables
add_subdirectory(apps)