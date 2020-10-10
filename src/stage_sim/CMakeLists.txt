find_package(stage REQUIRED)

add_executable(stagetest src/stest.cc)
target_link_libraries(stagetest ${STAGE_LIBRARIES})
target_include_directories(stagetest PRIVATE ${STAGE_INCLUDE_DIRS})