#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "xmotion::quadprogpp" for configuration "Release"
set_property(TARGET xmotion::quadprogpp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(xmotion::quadprogpp PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libquadprogpp.a"
  )

list(APPEND _cmake_import_check_targets xmotion::quadprogpp )
list(APPEND _cmake_import_check_files_for_xmotion::quadprogpp "${_IMPORT_PREFIX}/lib/libquadprogpp.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
