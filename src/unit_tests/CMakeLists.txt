include_directories(${GTEST_INCLUDE_DIRS})

# Add test code
set(GRAPH_GTEST_SRC
    src/graph/graph_bigfive_test.cpp
    src/graph/graph_type_test.cpp
    src/graph/graph_mod_test.cpp
    src/graph/graph_iter_test.cpp
    src/graph/graph_search_test.cpp
    src/graph/graph_search_inc_test.cpp
)

add_executable(gtest_all ${GRAPH_GTEST_SRC})
target_link_libraries(gtest_all gtest gtest_main graph)