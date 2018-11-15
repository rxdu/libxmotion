# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_markov_chain test_markov_chain.cpp)
target_link_libraries(test_markov_chain markov)