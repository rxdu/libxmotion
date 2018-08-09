# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(COLLISION_LIB_SRC
  src/collision.cpp
)
add_library(collision STATIC ${COLLISION_LIB_SRC})
#target_link_libraries(collision graph road reachability utilities)
target_include_directories(collision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
