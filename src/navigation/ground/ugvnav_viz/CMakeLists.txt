# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(UGVNAV_VIZ_LIB_SRC
    src/details/roadmap_draw.cpp
    src/details/vehicle_draw.cpp
    src/details/lattice_draw.cpp
    src/lattice_viz.cpp
    src/roadmap_viz.cpp
    src/reachability_viz.cpp
    # src/threat_field_viz.cpp
)
add_library(ugvnav_viz STATIC ${UGVNAV_VIZ_LIB_SRC})
target_link_libraries(ugvnav_viz decomp roadmap lattice reachability tfield lightviz)
target_include_directories(ugvnav_viz PUBLIC
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

