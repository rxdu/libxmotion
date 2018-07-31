# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(TRAFFICFLOW_LIB_SRC
    # src/field_search.cpp
    src/road_square_grid.cpp
    src/traffic_flow_map.cpp
    src/road_cost_map.cpp
    src/topogeo_graph.cpp
)
add_library(tflow STATIC ${TRAFFICFLOW_LIB_SRC})
target_link_libraries(tflow graph tfield decomp road)
target_include_directories(tflow PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# Tests
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()