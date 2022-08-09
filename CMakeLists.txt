cmake_minimum_required(VERSION 3.10.2)
project(robosw)

find_program(CCACHE_PROGRAM ccache)

if(CCACHE_PROGRAM)
    message(STATUS "Found ccache")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

# # Project Options
option(BUILD_TESTING "Build tests" ON)
option(ENABLE_TARGET_DESKTOP "Enable desktop-only modules" ON)
option(ENABLE_LOGGING "Enable logging" ON)
option(STATIC_CHECK "Enable static check" OFF)

# # Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(STATIC_CHECK)
    find_program(CPPCHECK cppcheck)
    if(CPPCHECK)
        message(STATUS "Found cppcheck")
        set(CMAKE_CXX_CPPCHECK cppcheck;--std=c++11;--enable=all)
    endif()
endif()

# # Additional cmake module path
set(USER_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${USER_CMAKE_PATH}/modules")
list(APPEND CMAKE_PREFIX_PATH "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/cmake")

# # Set compiler to use c++ 14 features
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # Use GNUInstallDirs to install libraries into correct locations on all platforms.
include(GNUInstallDirs)

# # Put all binary files into /bin and libraries into /lib
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

# # Chosse build type
set(default_build_type "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
        STRING "Choose the type of build." FORCE)

    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(ENABLE_LOGGING)
    message("-- Enable logging with spdlog -- Yes")
    message("-- Default log file storage location: librav/data/log")
else(ENABLE_LOGGING)
    message("-- Enable logging with spdlog -- No")
endif(ENABLE_LOGGING)

# # Enable test if requested
if(PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME AND BUILD_TESTING)
    enable_testing()
    include(GoogleTest)
    set(BUILD_TESTS ON)
    message(STATUS "Tests will be built")
else()
    set(BUILD_TESTS OFF)
    message(STATUS "Tests will not be built")
endif()

# # Add source directories
add_subdirectory(src)
add_subdirectory(extern)

# # Installation setup
# Offer the user the choice of overriding the installation directories
set(INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR} CACHE PATH "Installation directory for libraries")
set(INSTALL_BINDIR ${CMAKE_INSTALL_BINDIR} CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR} CACHE PATH "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
    set(DEF_INSTALL_CMAKEDIR CMake)
else()
    set(DEF_INSTALL_CMAKEDIR share/cmake/${PROJECT_NAME})
endif()
set(INSTALL_CMAKEDIR ${DEF_INSTALL_CMAKEDIR} CACHE PATH "Installation directory for CMake files")

# print installation paths
message(STATUS "Project will be installed to ${CMAKE_INSTALL_PREFIX} with 'make install'")
foreach(p LIB BIN INCLUDE CMAKE)
    file(TO_NATIVE_PATH ${CMAKE_INSTALL_PREFIX}/${INSTALL_${p}DIR} _path)
    message(STATUS " - To install ${p} components to ${_path}")
    unset(_path)
endforeach()

# export target configuration
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    roboswConfigVersion.cmake
    VERSION "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH}"
    COMPATIBILITY SameMajorVersion)

 install(EXPORT roboswTargets
     FILE roboswTargets.cmake
     NAMESPACE robosw::
     DESTINATION lib/cmake/robosw)

configure_file(cmake/roboswConfig.cmake.in roboswConfig.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/roboswConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/roboswConfigVersion.cmake"
    DESTINATION lib/cmake/robosw)

# Packaging support
set(CPACK_PACKAGE_VENDOR "Ruixiang Du")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Robot Software Packages")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/robosw")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "abc (>= 0.1.2), xyz (<< 0.3.4)")
set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Ruixiang Du (ruixiang.du@gmail.com)")
set(CPACK_SOURCE_IGNORE_FILES .git dist .*build.* /\\\\.DS_Store)
include(CPack)
