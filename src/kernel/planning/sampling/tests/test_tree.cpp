#include <iostream>
#include <cstdint>

#include "sampling/details/tree/tree.hpp"

using namespace librav;

int main()
{
    Tree<int32_t, double> tree;
    Tree<int32_t, double>::NodeType *nodes[8];

    for (int i = 0; i < 7; ++i)
        nodes[i] = new Tree<int32_t, double>::NodeType(i);

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
    tree.BFS();

    std::cout << "-----------" << std::endl;
    tree.DFSPreOrder(tree.GetRootNode());

    std::cout << "-----------" << std::endl;
    tree.DFSPostOrder(tree.GetRootNode());

    return 0;
}