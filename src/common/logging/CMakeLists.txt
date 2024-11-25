# Dependency libraries
find_package(Threads REQUIRED)

# Add libraries
set(LOGGING_LIB_SRC
    src/ctrl_logger.cpp
    src/default_logger.cpp
    src/logger_vendor_spdlog.cpp
)
add_library(logging STATIC ${LOGGING_LIB_SRC})
if (ENABLE_LOGGING)
  target_compile_definitions(logging PUBLIC "-DENABLE_LOGGING")
endif (ENABLE_LOGGING)
target_link_libraries(logging PUBLIC spdlog ${CMAKE_THREAD_LIBS_INIT})
target_include_directories(logging PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# Add executables
if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

install(TARGETS logging spdlog
    EXPORT xmotionTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})