# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(LIGHTVIEW_LIB_SRC
    src/lightview.cpp
)
add_library(lightview STATIC ${LIGHTVIEW_LIB_SRC})
target_link_libraries(lightview common utilities ${LCM_LIBRARIES})
target_include_directories(lightview PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# Add executables
if(BUILD_TESTS)
	add_subdirectory(tests)
endif()
