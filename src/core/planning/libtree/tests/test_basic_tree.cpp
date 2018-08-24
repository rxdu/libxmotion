#include <iostream>
#include <cstdint>

#include "tree/basic_tree.hpp"
#include "tree/algorithms/traversal.hpp"

using namespace librav;

int main()
{
    BasicTree<int32_t, double> tree;
    BasicTree<int32_t, double>::NodeType *nodes[8];

    for (int i = 0; i < 7; ++i)
        nodes[i] = new BasicTree<int32_t, double>::NodeType(i);

    tree.SetRootNode(nodes[0]);

    tree.ConnectNodes(nodes[0], nodes[1], 1);
    tree.ConnectNodes(nodes[0], nodes[2], 1);
    tree.ConnectNodes(nodes[0], nodes[3], 1);
    tree.ConnectNodes(nodes[1], nodes[4], 1);
    tree.ConnectNodes(nodes[2], nodes[5], 1);
    tree.ConnectNodes(nodes[2], nodes[6], 1);

    for (int i = 0; i < 7; ++i)
        std::cout << "node " << nodes[i]->state << " depth: " << nodes[i]->GetDepth() << std::endl;

    std::cout << "-----------" << std::endl;
    // tree.BFS();
    Traversal::BFS(&tree);

    std::cout << "-----------" << std::endl;
    // tree.DFSPreOrder(tree.GetRootNode());
    Traversal::DFSPreOrder(tree.GetRootNode());

    std::cout << "-----------" << std::endl;
    // tree.DFSPostOrder(tree.GetRootNode());
    Traversal::DFSPostOrder(tree.GetRootNode());

    return 0;
}