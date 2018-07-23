cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
project(VisApp)

## Cinder configurations
set(CMAKE_VERBOSE_MAKEFILE ON)
set(cinder_DIR "${CMAKE_BINARY_DIR}/lib")

get_filename_component(CINDER_PATH "${CMAKE_SOURCE_DIR}/third_party/cinder" ABSOLUTE)
get_filename_component(APP_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)

include("${CINDER_PATH}/proj/cmake/modules/cinderMakeApp.cmake")

## Make Cinder app
ci_make_app(
	SOURCES     ${APP_PATH}/src/vis_app.cpp
	CINDER_PATH ${CINDER_PATH}
)
