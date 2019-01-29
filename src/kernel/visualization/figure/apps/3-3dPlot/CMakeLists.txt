cmake_minimum_required( VERSION 2.8 ) 
project( Examples/3-3dPlot )

add_definitions( -std=gnu++11 )

set( CMAKE_MODULE_PATH  ${CMAKE_CURRENT_SOURCE_DIR}/../../../../CMake/Modules )   

find_package( Eigen3 REQUIRED )
find_package( MathGL2 2.0.0 REQUIRED )
find_package( Figure REQUIRED )

include_directories( ${EIGEN_INCLUDE_DIR} ${MATHGL2_INCLUDE_DIRS} ${FIGURE_INCLUDE_DIR} )
add_executable( main_3dplot main.cpp )
target_link_libraries( main_3dplot ${MATHGL2_LIBRARIES} ${FIGURE_LIBRARY} )

