# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(ROAD_LIB_SRC
    src/road_map.cpp
    src/road_geometry.cpp
    src/road_coordinate.cpp
)
add_library(road STATIC ${ROAD_LIB_SRC})
target_link_libraries(road decomp lanelet utilities)
target_include_directories(road PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

add_subdirectory(tests)

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

## tests
#add_executable(graph_example graph_example.cpp)
#target_link_libraries(graph_example vis map geometry)
