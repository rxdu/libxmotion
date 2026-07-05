#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "xmotion::xmBase" for configuration "Release"
set_property(TARGET xmotion::xmBase APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(xmotion::xmBase PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libxmBase.a"
  )

list(APPEND _cmake_import_check_targets xmotion::xmBase )
list(APPEND _cmake_import_check_files_for_xmotion::xmBase "${_IMPORT_PREFIX}/lib/libxmBase.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
