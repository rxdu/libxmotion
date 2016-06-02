CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT( octomap-distribution )

ENABLE_TESTING()  # enable CTest environment of subprojects

SET( BASE_DIR ${CMAKE_BINARY_DIR} )
SET( CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR} )

option(BUILD_OCTOVIS_SUBPROJECT "Build targets from subproject octovis" OFF)
option(BUILD_DYNAMICETD3D_SUBPROJECT  "Build targets from subproject dynamicEDT3D" OFF)

ADD_SUBDIRECTORY( octomap )

if(BUILD_OCTOVIS_SUBPROJECT)
	ADD_SUBDIRECTORY( octovis )
endif()		

if(BUILD_DYNAMICETD3D_SUBPROJECT)	
	ADD_SUBDIRECTORY( dynamicEDT3D )
endif()		


