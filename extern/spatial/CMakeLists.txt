cmake_minimum_required(VERSION 3.0)
project(spatial)

add_library(spatial INTERFACE)
target_include_directories(spatial INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)