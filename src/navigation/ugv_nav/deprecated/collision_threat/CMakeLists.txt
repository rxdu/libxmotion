# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(TMAP_LIB_SRC
    src/motion_model.cpp
    src/collision_threat.cpp
    src/threat_ranking.cpp
)
add_library(cthreat STATIC ${TMAP_LIB_SRC})
target_link_libraries(cthreat decomp graph tree roadmap trafficmap tfield lattice lattice_planner ugvnav_viz lightviz)
target_include_directories(cthreat PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

# TEMPLATE, TO BE UPDATED
# # 'make install' to the correct locations (provided by GNUInstallDirs).
# install(TARGETS lib EXPORT MyLibraryConfig
# ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
# LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
# RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
# install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# # This makes the project importable from the install directory
# # Put config file in per-project dir (name MUST match), can also
# # just go into 'cmake'.
# install(EXPORT MyLibraryConfig DESTINATION share/MyLibrary/cmake)

# # This makes the project importable from the build directory
# export(TARGETS lib FILE MyLibraryConfig.cmake)

