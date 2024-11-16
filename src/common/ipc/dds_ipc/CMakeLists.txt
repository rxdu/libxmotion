## Dependency libraries
find_package(CycloneDDS REQUIRED)

## Add libraries
add_library(dds_ipc STATIC
        src/dds_topic.cpp
        src/dds_publisher.cpp
        src/dds_participant.cpp)
target_link_libraries(dds_ipc PUBLIC CycloneDDS::ddsc)
target_include_directories(dds_ipc PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src)

if (BUILD_TESTS)
    add_subdirectory(test)
endif ()
