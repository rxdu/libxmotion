# Dependency libraries
find_package(Threads REQUIRED)

# Add libraries
add_library(event STATIC
    src/async_event_dispatcher.cpp
    src/event_dispatcher.cpp)
target_link_libraries(event PUBLIC xmotion::xmSigma)
target_include_directories(event PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# Add executables
if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

# Internal build-only library — not part of xmNabla's public/exported API, so it
# is intentionally not installed/exported. Promote to xmSigma if a second
# component ever needs it (see ADR 0001 — foundation charter).