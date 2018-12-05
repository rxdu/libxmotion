# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(QUADCON_LIB_SRC
    # # state estimator
    # src/att_estimator.cpp
    # # data types
    # src/quad_state.cpp
    # # controller
    # src/att_quat_con.cpp
    # src/pos_quat_con.cpp
    # # actuator
    # src/quad_mixer.cpp    
    src/rccar_odesim.cpp
)
add_library(rccar_ctrl STATIC ${QUADCON_LIB_SRC})
target_link_libraries(rccar_ctrl ctrl_if common model utils datalink)
target_include_directories(rccar_ctrl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# Add executables
if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
