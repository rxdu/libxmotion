/* 
 * search.hpp
 * 
 * Created on: Aug 24, 2018 09:45
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SEARCH_HPP
#define SEARCH_HPP

#include <iostream>

namespace librav
{
struct Search
{
    // template <typename TreeNode, typename State>
    // static TreeNode *DFSPreOrder(TreeNode *root, State state)
    // {
    //     if (root == nullptr)
    //     {
    //         std::cout << "returning nullptr" << std::endl;
    //         return nullptr;
    //     }
    //     // std::cout << root->state << std::endl;
    //     std::cout << "none empty root " << root->state << " vs " << state << std::endl;
    //     if (root->state == state)
    //     {
    //         std::cout << "returning" << std::endl;
    //         return root;
    //     }
    //     for (auto &nd : root->children)
    //         return DFSPreOrder(nd.first, state);
    // }

    // template <typename TreeNode>
    // static void DFSPostOrder(TreeNode *root)
    // {
    //     if (root == nullptr)
    //         return;
    //     for (auto &nd : root->children)
    //         DFSPreOrder(nd.first);
    //     std::cout << root->state << std::endl;
    // }

    template <typename Tree, typename State>
    static typename Tree::NodeType *BFS(Tree *tree, State state)
    {
        typename Tree::NodeType *node = tree->GetRootNode();

        std::queue<typename Tree::NodeType *> q;
        q.push(node);
        while (!q.empty())
        {
            node = q.front();
            q.pop();
            // std::cout << node->state << std::endl;
            if (node->state == state)
                return node;
            for (auto &nd : node->children)
                q.push(nd.first);
        }
    }
};
} // namespace librav

#endif /* SEARCH_HPP */
