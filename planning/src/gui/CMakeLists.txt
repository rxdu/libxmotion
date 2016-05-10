# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)

# Dependency libraries
find_package(OpenCV REQUIRED)

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
  planner_gui.cpp
  mainwindow.cpp
  image_label.cpp
  ${MOC_FILES}
)

set(FORM_FILES 
  mainwindow.ui
)
qt5_wrap_cpp(MOC_SRC_H
  ${HEADER_FILES}
)

qt5_wrap_ui(MOC_GUI_H 
  ${FORM_FILES}
)

qt5_generate_moc(planner_gui.cpp quad_gui.moc)
add_executable(planner_gui ${SOURCE_FILES} ${MOC_SRC_H} ${MOC_GUI_H})
set_target_properties(planner_gui PROPERTIES COMPILE_FLAGS "-std=c++11 -g")
target_link_libraries(planner_gui map squaregrid vis ${OpenCV_LIBS} ${Boost_LIBRARIES} Qt5::Widgets)
