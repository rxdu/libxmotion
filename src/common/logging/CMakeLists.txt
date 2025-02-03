# Dependency libraries
find_package(Threads REQUIRED)

if (USE_SYS_SPDLOG)
  find_package(spdlog REQUIRED)
endif ()

# Add libraries
set(LOGGING_LIB_SRC
    src/ctrl_logger.cpp
    src/default_logger.cpp
    src/logger_vendor_spdlog.cpp)
add_library(logging STATIC ${LOGGING_LIB_SRC})
if (ENABLE_LOGGING)
  target_compile_definitions(logging PUBLIC "-DENABLE_LOGGING")
  if (USE_SYS_SPDLOG)
    target_link_libraries(logging PUBLIC spdlog::spdlog)
  else ()
    target_link_libraries(logging PUBLIC spdlog)
  endif ()
endif (ENABLE_LOGGING)
target_link_libraries(logging PUBLIC ${CMAKE_THREAD_LIBS_INIT})
target_include_directories(logging PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# Add executables
if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

if (USE_SYS_SPDLOG)
  set(libs logging)
else ()
  set(libs logging spdlog)
endif ()

install(TARGETS ${libs}
    EXPORT xmotionTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})