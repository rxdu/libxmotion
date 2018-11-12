# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(TFIELD_LIB_SRC
    src/vehicle_estimation.cpp
    src/dynamic_threat_model.cpp
    src/vehicle_threat.cpp
    src/threat_field.cpp
)
add_library(tfield STATIC ${TFIELD_LIB_SRC})
target_link_libraries(tfield common decomp graph traffic reachability)
target_include_directories(tfield PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# add_subdirectory(apps)

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

