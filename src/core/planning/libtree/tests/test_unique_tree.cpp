#include <iostream>
#include <cstdint>

#include "tree/unique_tree.hpp"
#include "tree/algorithms/traversal.hpp"
#include "tree/algorithms/search.hpp"

using namespace librav;

int main()
{
    UniqueTree<int32_t, double> tree;

    // UniqueTree<int32_t, double>::NodeType *nodes[7];
    // for (int i = 0; i < 7; ++i)
    //     nodes[i] = new UniqueTree<int32_t, double>::NodeType(i);
    // tree.SetRootNode(nodes[0]);
    // tree.ConnectNodes(nodes[0], nodes[1], 1);
    // tree.ConnectNodes(nodes[0], nodes[2], 1);
    // tree.ConnectNodes(nodes[0], nodes[3], 1);
    // tree.ConnectNodes(nodes[1], nodes[4], 1);
    // tree.ConnectNodes(nodes[2], nodes[5], 1);
    // tree.ConnectNodes(nodes[2], nodes[6], 1);
    // for (int i = 0; i < 7; ++i)
    //     std::cout << "node " << nodes[i]->state << " depth: " << nodes[i]->GetDepth() << std::endl;

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

    return 0;
}