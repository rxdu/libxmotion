# Dependency libraries
# find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(SAMPLING_LIB_SRC
    src/rand_num_gen.cpp
)
add_library(sampling ${SAMPLING_LIB_SRC})
# target_link_libraries(sampling common)
# target_sources(sampling INTERFACE ${SAMPLING_LIB_SRC})
target_include_directories(sampling PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  private src)

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
