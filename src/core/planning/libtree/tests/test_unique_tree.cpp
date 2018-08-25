#include <iostream>
#include <cstdint>

#include "tree/unique_tree.hpp"
#include "tree/algorithms/traversal.hpp"
#include "tree/algorithms/search.hpp"

using namespace librav;

int main()
{
    UniqueTree<int32_t, double> tree;

    tree.ConnectNodes(0, 1, 1);
    tree.ConnectNodes(0, 2, 1);
    tree.ConnectNodes(0, 3, 1);
    tree.ConnectNodes(1, 4, 1);
    tree.ConnectNodes(2, 5, 1);
    tree.ConnectNodes(2, 6, 1);

    tree.DisconnectNodes(2, 6);

    // std::cout << "-----------" << std::endl;
    Traversal::BFS(&tree);

    // std::cout << "-----------" << std::endl;
    // Traversal::DFSPreOrder(tree.GetRootNode());

    // std::cout << "-----------" << std::endl;
    // Traversal::DFSPostOrder(tree.GetRootNode());

    // auto nd = Search::BFS(&tree, 2);
    // if (nd != nullptr)
    //     std::cout << "found: " << nd->state << " depth: " << nd->GetDepth() << std::endl;
    // else
    //     std::cout << "not found" << std::endl;

    std::cout << "--- copy ---" << std::endl;
    UniqueTree<int32_t, double> tree2(tree);
    Traversal::BFS(&tree2);

    std::cout << "--- assign ---" << std::endl;
    UniqueTree<int32_t, double> tree3;
    tree3 = tree;
    Traversal::BFS(&tree3);

    std::cout << "--- move ---" << std::endl;
    UniqueTree<int32_t, double> tree4(std::move(tree2));
    Traversal::BFS(&tree4);

    std::cout << "--- move assign ---" << std::endl;
    UniqueTree<int32_t, double> tree5;
    tree5 = std::move(tree3);
    Traversal::BFS(&tree5);

    return 0;
}