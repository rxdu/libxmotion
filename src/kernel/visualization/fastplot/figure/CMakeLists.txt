## Dependency libraries
# find and include Eigen
find_package(Eigen3 REQUIRED)

# find and inlude MathGl
find_package(MathGL2 2.0.0 REQUIRED)

#  patch mgl2/config.h
# file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/include/mathgl_patched_headers/mgl2)
# file(READ ${MATHGL2_INCLUDE_DIRS}/mgl2/config.h MATHGL_PATCHED_CONFIG_H)
# STRING(REGEX REPLACE "#define MGL_HAVE_TYPEOF[ \t]+1" "#define MGL_HAVE_TYPEOF 0 // patched"
#                      MATHGL_PATCHED_CONFIG_H ${MATHGL_PATCHED_CONFIG_H})
# STRING(REGEX REPLACE "#define MGL_HAVE_C99_COMPLEX[ \t]+1" "#define MGL_HAVE_C99_COMPLEX 0 // patched"
#                      MATHGL_PATCHED_CONFIG_H ${MATHGL_PATCHED_CONFIG_H})
# file(WRITE ${CMAKE_BINARY_DIR}/include/mathgl_patched_headers/mgl2/config.h ${MATHGL_PATCHED_CONFIG_H} )


## Add libraries
set(FIGURE_LIB_SRC
	src/figure.cpp
)
add_library(figure STATIC ${FIGURE_LIB_SRC})
target_link_libraries(figure ${MATHGL2_LIBRARIES})
target_include_directories(figure PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include/mathgl_patched_headers/>
    $<BUILD_INTERFACE:${MATHGL2_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:include>	
	PRIVATE src)

add_subdirectory(apps)

# Add executables
if(BUILD_TESTS)
	# add_subdirectory(tests)
endif()