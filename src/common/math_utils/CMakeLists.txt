add_library(math_utils STATIC src/matrix.cpp)
target_link_libraries(math_utils PUBLIC xmotion::logging stb)
target_include_directories(math_utils PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

if(BUILD_TESTS)
  add_subdirectory(test)
endif()

# Internal build-only library (used only by the disabled mapping module) — not
# part of xmNabla's public/exported API, so intentionally not installed/exported.
# This also keeps its public `stb` dependency out of xmNablaTargets.
