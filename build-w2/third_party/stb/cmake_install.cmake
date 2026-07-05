# Install script for directory: /home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/stb" TYPE FILE FILES
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_c_lexer.h"
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_connected_components.h"
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_divide.h"
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_ds.h"
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_dxt.h"
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_easy_font.h"
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_herringbone_wang_tile.h"
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_hexwave.h"
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_image.h"
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_image_resize2.h"
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_image_write.h"
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_include.h"
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_leakcheck.h"
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_perlin.h"
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_rect_pack.h"
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_sprintf.h"
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_textedit.h"
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_tilemap_editor.h"
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_truetype.h"
    "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/third_party/stb/stb_voxel_render.h"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/rdu/RduWs/robotics_toolbox/xmotion/components/navigation/build-w2/third_party/stb/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
