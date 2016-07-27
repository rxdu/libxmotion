# Dependency libraries
find_package(OpenCV REQUIRED)

find_package(VTK 7.0 REQUIRED COMPONENTS
  vtkCommonCore
  vtkGUISupportQt
  vtkIOInfovis
  vtkViewsInfovis)
include(${VTK_USE_FILE}) 

# Add non-Qt libraries
#set(VIS_LIB_SRC
#    graph_vis.cpp
#)
#add_library(vis STATIC ${VIS_LIB_SRC})
#target_link_libraries(vis map ${OpenCV_LIBS})

# Add non-Qt executables
#add_executable(test_graph test_graph.cpp)
#target_link_libraries(test_graph qtree vis map ${OpenCV_LIBS})

## Qt configurations
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library
find_package(Qt5Widgets)

# Add the include directories for the Qt 5 Widgets module to
# the compile lines.
include_directories(${Qt5Widgets_INCLUDE_DIRS})

# Use the compile definitions defined in the Qt 5 Widgets module
add_definitions(${Qt5Widgets_DEFINITIONS})

# Add compiler flags for building executables (-fPIE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

## Declare a QT C++ executable
set(HEADER_FILES 
  mainwindow.h
  image_label.h
  vtk_viewer.h
  map_viewer.h
)

set(SOURCE_FILES
  gui_main.cpp
  mainwindow.cpp
  image_label.cpp
  vtk_viewer.cpp
  map_viewer.cpp
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

# Add Qt executables
add_executable(srcl_gui ${SOURCE_FILES} ${MOC_SRC_H} ${MOC_GUI_H} ${MOC_RES_H})
set_target_properties(srcl_gui PROPERTIES COMPILE_FLAGS "-std=c++11 -g")
target_link_libraries(srcl_gui map squaregrid vis ${OpenCV_LIBS} ${VTK_LIBRARIES} Qt5::Widgets)
