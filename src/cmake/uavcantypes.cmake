# Macros for automatically compiling UAVCAN types into C++ headers.
#
# The primary macro is:
#     uavcantypes_build()
#
# ----
# File: uavcantypes.cmake
# To be updated: https://blog.kangz.net/posts/2016/05/26/integrating-a-code-generator-with-cmake/

cmake_minimum_required(VERSION 2.6.0)

set(UAVCAN_DSDL_COMPILER ${CMAKE_SOURCE_DIR}/libraries/uavcan/libuavcan/dsdl_compiler/libuavcan_dsdlc)
set(UAVCAN_DSDL_TYPEDEF_DIR ${CMAKE_SOURCE_DIR}/uavcantypes/pixcar)
#set(UAVCAN_DSDL_TYPEDEF_DIR ${CMAKE_SOURCE_DIR}/../../firmware/flight/UAVCAN/pixcar)
set(UAVCAN_DSDL_OUTPUT_DIR ${CMAKE_BINARY_DIR}/include) 

function(uavcangen)
    execute_process(COMMAND ${UAVCAN_DSDL_COMPILER} -O ${UAVCAN_DSDL_OUTPUT_DIR} ${UAVCAN_DSDL_TYPEDEF_DIR})
endfunction()

macro(uavcantypes_build)
    uavcangen()
endmacro()
