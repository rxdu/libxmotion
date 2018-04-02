# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(LANELET_LIB_SRC
  src/LLTree.cpp
  src/Lanelet.cpp
  src/llet_xml.cpp
  src/Attribute.cpp
  src/LineStrip.cpp
  src/LaneletMap.cpp
  src/LaneletBase.cpp
  # src/lanelet_tests.cpp
  src/CompoundLanelet.cpp
  src/RegulatoryElement.cpp
)
add_library(lanelet STATIC ${LANELET_LIB_SRC})
target_link_libraries(lanelet lanelet_common pugixml)
target_include_directories(lanelet PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# add_executable(demo src/demo.cpp)
# target_link_libraries(demo lanelet)

