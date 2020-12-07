# Dependency libraries
find_package(Eigen3 REQUIRED)

# Add library
add_library(image_io STATIC src/image_io.cpp)
target_link_libraries(image_io eigenfileio)
target_compile_options(image_io PUBLIC "-Wno-format-truncation")
target_include_directories(image_io PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> 
    PRIVATE src)

if(BUILD_TESTS)
	add_subdirectory(test)
endif()