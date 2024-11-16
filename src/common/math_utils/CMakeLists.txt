find_package(Boost REQUIRED filesystem)

add_library(math_utils STATIC src/matrix.cpp)
target_link_libraries(math_utils PUBLIC logging stb Boost::filesystem)
target_include_directories(math_utils PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

if(BUILD_TESTS)
  add_subdirectory(test)
endif()

## installation configuration
install(TARGETS math_utils
        EXPORT xmotionTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
