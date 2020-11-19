## Test modules
# graph
set(GRAPH_GTEST_SRC
    graph/graph_bigfive_test.cpp
    graph/graph_type_test.cpp
    graph/graph_mod_test.cpp
    graph/graph_iter_test.cpp
    graph/graph_search_test.cpp
    graph/graph_search_inc_test.cpp
    graph/tree_bigfive_test.cpp
    graph/tree_mod_test.cpp
)
set(GRAPH_GTEST_LIB graph)
add_executable(gtest_graph ${GRAPH_GTEST_SRC})
target_link_libraries(gtest_graph gtest gtest_main ${GRAPH_GTEST_LIB})

# geometry
set(GEOMETRY_GTEST_SRC
    geometry/cspline_test.cpp
    geometry/polynomial_test.cpp
)
set(GEOMETRY_GTEST_LIB geometry)
add_executable(gtest_geometry ${GEOMETRY_GTEST_SRC})
target_link_libraries(gtest_geometry gtest gtest_main ${GEOMETRY_GTEST_LIB})

# markov
set(MARKOV_GTEST_SRC
    markov/markov_chain_test.cpp
    markov/markov_chain_x_test.cpp
)
set(MARKOV_GTEST_LIB markov)
add_executable(gtest_markov ${MARKOV_GTEST_SRC})
target_link_libraries(gtest_markov gtest gtest_main ${MARKOV_GTEST_LIB})

# Group all tests 
set(ALL_GTEST_SRC
    ${GRAPH_GTEST_SRC} 
    ${GEOMETRY_GTEST_SRC}
    ${MARKOV_GTEST_SRC}
)
set(ALL_GTEST_LIBS
    ${GRAPH_GTEST_LIB}
    ${GEOMETRY_GTEST_LIB}
    ${MARKOV_GTEST_LIB}
)
add_executable(gtest_all ${ALL_GTEST_SRC})
target_link_libraries(gtest_all gtest gtest_main ${ALL_GTEST_LIBS})