#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "xmotion::estimation" for configuration "Release"
set_property(TARGET xmotion::estimation APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(xmotion::estimation PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libestimation.a"
  )

list(APPEND _cmake_import_check_targets xmotion::estimation )
list(APPEND _cmake_import_check_files_for_xmotion::estimation "${_IMPORT_PREFIX}/lib/libestimation.a" )

# Import target "xmotion::pid" for configuration "Release"
set_property(TARGET xmotion::pid APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(xmotion::pid PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpid.a"
  )

list(APPEND _cmake_import_check_targets xmotion::pid )
list(APPEND _cmake_import_check_files_for_xmotion::pid "${_IMPORT_PREFIX}/lib/libpid.a" )

# Import target "xmotion::kinematics" for configuration "Release"
set_property(TARGET xmotion::kinematics APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(xmotion::kinematics PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libkinematics.a"
  )

list(APPEND _cmake_import_check_targets xmotion::kinematics )
list(APPEND _cmake_import_check_files_for_xmotion::kinematics "${_IMPORT_PREFIX}/lib/libkinematics.a" )

# Import target "xmotion::geometry" for configuration "Release"
set_property(TARGET xmotion::geometry APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(xmotion::geometry PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libgeometry.a"
  )

list(APPEND _cmake_import_check_targets xmotion::geometry )
list(APPEND _cmake_import_check_files_for_xmotion::geometry "${_IMPORT_PREFIX}/lib/libgeometry.a" )

# Import target "xmotion::sampling" for configuration "Release"
set_property(TARGET xmotion::sampling APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(xmotion::sampling PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libsampling.a"
  )

list(APPEND _cmake_import_check_targets xmotion::sampling )
list(APPEND _cmake_import_check_files_for_xmotion::sampling "${_IMPORT_PREFIX}/lib/libsampling.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
