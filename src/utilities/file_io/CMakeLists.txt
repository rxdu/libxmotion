find_package(Boost REQUIRED filesystem)

add_library(file_io INTERFACE)
target_link_libraries(file_io INTERFACE ${Boost_LIBRARIES})
target_include_directories(file_io INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()