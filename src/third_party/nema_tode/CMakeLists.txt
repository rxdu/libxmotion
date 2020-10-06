cmake_minimum_required(VERSION 3.0.0)
project(nema_tode)

## Find dependencies
# find_package(Boost REQUIRED system filesystem)
# include_directories(${Boost_INCLUDE_DIRS})

# Add libraries
set(NEMAPARSE_LIB_SRC
    src/GPSFix.cpp
    src/GPSService.cpp
    src/NMEACommand.cpp
    src/NMEAParser.cpp
    src/NumberConversion.cpp
)
add_library(nemaparse STATIC ${NEMAPARSE_LIB_SRC})
# target_link_libraries(nemaparse ${Boost_LIBRARIES})
target_include_directories(nemaparse PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Add tests
# if(BUILD_TESTS)
#   add_subdirectory(tests)
# endif()
