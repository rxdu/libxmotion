# to use this Figure library :
#                 find_package( FIGURE REQUIRED )
#                 include_directories( ${FIGURE_INCLUDE_DIR} )
#                 add_executable( exec_name exec_source.cpp )
#                 target_link_libraries( exec_name Figure )

cmake_minimum_required( VERSION 2.8 )
project( FigureClass )

# use c++11
if(CMAKE_VERSION VERSION_GREATER 3.1.0 OR CMAKE_VERSION VERSION_EQUAL 3.1.0)
    # use c++11
    # only valid for new platforms
    set(CMAKE_CXX_STANDARD 11)
else()
    if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
	# for older cmake versions
	# (note, this CXX flag is only valid for clang and gcc, for MSVC it is not needed)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    endif()
endif()
# obsulete since we disabled MGL_HAVE_C99_COMPLEX, MGL_HAVE_TYPEOF
#add_definitions( -Wall -std=gnu++11 -fext-numeric-literals )

set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../../../CMake/Modules/ )

# set all header files in variable HEADER_FILES
set( HEADER_FILES ../src/figure.hpp
                  ../src/FigureConfig.hpp
                  ../src/MglLabel.hpp
                  ../src/MglPlot.hpp
                  ../src/MglStyle.hpp )

# find and include Eigen
find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

# find and inlude MathGl
find_package( MathGL2 2.0.0 REQUIRED )
#  patch mgl2/config.h
file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/mathgl_patched_headers/mgl2)
file(READ ${MATHGL2_INCLUDE_DIRS}/mgl2/config.h MATHGL_PATCHED_CONFIG_H)
STRING(REGEX REPLACE "#define MGL_HAVE_TYPEOF[ \t]+1" "#define MGL_HAVE_TYPEOF 0 // patched"
                     MATHGL_PATCHED_CONFIG_H ${MATHGL_PATCHED_CONFIG_H})
STRING(REGEX REPLACE "#define MGL_HAVE_C99_COMPLEX[ \t]+1" "#define MGL_HAVE_C99_COMPLEX 0 // patched"
                     MATHGL_PATCHED_CONFIG_H ${MATHGL_PATCHED_CONFIG_H})

file(WRITE ${PROJECT_BINARY_DIR}/mathgl_patched_headers/mgl2/config.h ${MATHGL_PATCHED_CONFIG_H} )
include_directories(${PROJECT_BINARY_DIR}/mathgl_patched_headers/) 
include_directories(${MATHGL2_INCLUDE_DIRS})

# build library
add_library( Figure ../src/figure.cpp )
target_link_libraries( Figure ${MATHGL2_LIBRARIES} )

# install library and header files
install( TARGETS Figure 
         ARCHIVE DESTINATION lib
         LIBRARY DESTINATION lib )
install( FILES ${HEADER_FILES} 
         DESTINATION include/figure )
