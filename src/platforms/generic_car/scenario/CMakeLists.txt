# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(SCENARIO_LIB_SRC
    src/left_right_turn.cpp
)
add_library(scenario STATIC ${SCENARIO_LIB_SRC})
target_link_libraries(scenario common geometry graph planner)
target_include_directories(scenario PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

add_subdirectory(tests)