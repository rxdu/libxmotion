# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(TRANKING_LIB_SRC
    src/topogeo_graph.cpp
    src/motion_model.cpp
    # src/threat_ranking.cpp
)
add_library(rank STATIC ${TRANKING_LIB_SRC})
target_link_libraries(rank decomp graph roadmap tfield lattice lattice_planner lightviz)
target_include_directories(rank PUBLIC
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

