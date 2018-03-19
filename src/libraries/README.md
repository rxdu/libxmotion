## Changes made to third-party libraries

### octomap-1.8.0

octomap

* set "CMAKE_INSTALL_PREFIX" to be "${CMAKE_BINARY_DIR}" in octomap-1.8.0/octomap/CMakeLists.txt
* set "BASE_DIR" to be ${CMAKE_BINARY_DIR} in octomap-1.8.0/octomap/CMakeLists.txt
* commented out the original define of "BASE_DIR" in octomap-1.8.0/octomap/CMakeLists.txt at line 34
* changed binary output directory from "${BASE_DIR}/bin" to "${PROJECT_BINARY_DIR}/bin" in the octomap-1.8.0/octomap/CMakeLists.txt
* commented out conditions for "add_dependencies()" for octovis and dynamicsEDT3D (because same target is defined in ompl)
* commented out the target "uninstall" in octomap-1.8.0/octomap/CMakeLists.txt at line 76-77

octovis

* changed the HINTS location in octomap-1.8.0/octovis/CMakeLists.txt at line 40,41

### ompl-1.2.0

* changed the library output directory from "${PROJECT_BINARY_DIR}/lib" to "${CMAKE_BINARY_DIR}/lib" in the ompl-1.2.0/CMakeLists.txt at line 43

### spdlog

* commit: e6cbc22da53bc807a6172a99cd6dc75534926b99

### dubins_curves

* minimal version for cleaner integration: https://github.com/rxdu/dubins_curves/tree/minimal