## Changes made to third-party libraries

### eigen: unchanged

### octomap-1.8.0

* set "CMAKE_INSTALL_PREFIX" to be "${CMAKE_BINARY_DIR}" in octomap-1.8.0/CMakeLists.txt
* set "BASE_DIR" to be ${CMAKE_BINARY_DIR} in octomap-1.8.0/CMakeLists.txt
* commented out the original define of "BASE_DIR" in octomap-1.8.0/octomap/CMakeLists.txt at line 34
* changed binary output directory from "${BASE_DIR}/bin" to "${PROJECT_BINARY_DIR}/bin" in the octomap/CMakeLists.txt
* commented out conditions for "add_dependencies()" for octovis and dynamicsEDT3D (because same target is defined in ompl)
* commented out the target "uninstall" in octomap/CMakeLists.txt at line 76-77

### g3log_srcl

* refer to "change_forl_srcl.txt" inside the library folder

### ompl-1.2.0

* changed the library output directory from "${PROJECT_BINARY_DIR}/lib" to "${CMAKE_BINARY_DIR}/lib" in the ompl-1.2.0/CMakeLists.txt at line 43

### SMP (Sertac Karaman)

* Added CMakeLists.txt to build external libraries
