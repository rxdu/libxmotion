#include <iostream>

#include "rrt/tree.hpp"

using namespace librav;

int main()
{
    Tree<double, double> tree;

    Tree<double, double>::NodeType *start = new Tree<double, double>::NodeType(0.1);
    tree.SetRootNode(start);
    std::cout << "root depth: " << start->GetDepth() << std::endl;

    Tree<double, double>::NodeType *node1 = new Tree<double, double>::NodeType(0.2);
    tree.ConnectNodes(tree.GetRootNode(), node1, 1);
    std::cout << "node1 depth: " << node1->GetDepth() << std::endl;

    Tree<double, double>::NodeType *node2 = new Tree<double, double>::NodeType(0.21);
    tree.ConnectNodes(tree.GetRootNode(), node2, 2);
    std::cout << "node2 depth: " << node2->GetDepth() << std::endl;

    Tree<double, double>::NodeType *node3 = new Tree<double, double>::NodeType(0.3);
    tree.ConnectNodes(node1, node3, 3);
    std::cout << "node3 depth: " << node3->GetDepth() << std::endl;

    std::cout << "-----------" << std::endl;
    tree.BFS();

    return 0;
}