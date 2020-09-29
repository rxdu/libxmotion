# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(PUGIXML_LIB_SRC
  src/pugixml.cpp
)
add_library(pugixml STATIC ${PUGIXML_LIB_SRC})
# target_link_libraries(pugixml common)
target_include_directories(pugixml PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


