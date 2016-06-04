cmake_minimum_required(VERSION 2.8.11)
project(srcl_gui)

## Set compiler to use c++ 11 features
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 or C++11 support. Please use a different C++ compiler.")
endif()

# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/planning/src)

# Dependency libraries
find_package(OpenCV REQUIRED)

find_package(VTK 7.0 REQUIRED COMPONENTS
  vtkCommonCore
  vtkGUISupportQt
  vtkIOInfovis
  vtkViewsInfovis)
include(${VTK_USE_FILE}) 

# Add libraries
#set(VIS_LIB_SRC
#    graph_vis.cpp
#)
#add_library(vis STATIC ${VIS_LIB_SRC})
#target_link_libraries(vis map ${OpenCV_LIBS})

# Add executables
#add_executable(test_graph test_graph.cpp)
#target_link_libraries(test_graph qtree vis map ${OpenCV_LIBS})

## Qt configurations
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library
find_package(Qt5Widgets)
find_package(Boost REQUIRED COMPONENTS system)

# Add the include directories for the Qt 5 Widgets module to
# the compile lines.
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

# Use the compile definitions defined in the Qt 5 Widgets module
add_definitions(${Qt5Widgets_DEFINITIONS})

# Add compiler flags for building executables (-fPIE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

## Declare a QT C++ executable
set(HEADER_FILES 
  mainwindow.h
  image_label.h
)

set(SOURCE_FILES
  gui_main.cpp
  mainwindow.cpp
  image_label.cpp
  ${MOC_FILES}
)

set(FORM_FILES 
  mainwindow.ui
)

set(RES_FILES 
  icon_res.qrc
)

qt5_wrap_cpp(MOC_SRC_H
  ${HEADER_FILES}
)

qt5_wrap_ui(MOC_GUI_H 
  ${FORM_FILES}
)

qt5_add_resources(MOC_RES_H 
  ${RES_FILES}
)

qt5_generate_moc(gui_main.cpp gui_main.moc)
add_executable(gui_main ${SOURCE_FILES} ${MOC_SRC_H} ${MOC_GUI_H} ${MOC_RES_H})
set_target_properties(gui_main PROPERTIES COMPILE_FLAGS "-std=c++11 -g")
target_link_libraries(gui_main map2d squaregrid vis ${OpenCV_LIBS} ${Boost_LIBRARIES} ${VTK_LIBRARIES} Qt5::Widgets)
