/* 
 * traversal.hpp
 * 
 * Created on: Aug 24, 2018 05:47
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAVERSAL_HPP
#define TRAVERSAL_HPP

#include <iostream>
#include <vector>

namespace librav
{
struct Traversal
{
    template <typename TreeNode>
    static void DFSPreOrder(TreeNode *root)
    {
        if (root == nullptr)
            return;
        std::cout << root->state << std::endl;
        for (auto &nd : root->children)
            DFSPreOrder(nd.first);
    }

    template <typename TreeNode>
    static void DFSPostOrder(TreeNode *root)
    {
        if (root == nullptr)
            return;
        for (auto &nd : root->children)
            DFSPreOrder(nd.first);
        std::cout << root->state << std::endl;
    }

    template <typename Tree>
    static void BFS(Tree *tree)
    {
        typename Tree::NodeType *root = tree->GetRootNode();

        std::queue<typename Tree::NodeType *> q;
        q.push(root);
        while (!q.empty())
        {
            root = q.front();
            q.pop();
            std::cout << root->state << std::endl;
            for (auto &nd : root->children)
                q.push(nd.first);
        }
    }

    template <typename Tree>
    static std::vector<typename Tree::StateType> GetAllStatesBF(Tree *tree)
    {
        typename Tree::NodeType *root = tree->GetRootNode();

        std::vector<typename Tree::StateType> states;
        std::queue<typename Tree::NodeType *> q;
        q.push(root);
        while (!q.empty())
        {
            root = q.front();
            q.pop();
            // std::cout << root->state << std::endl;
            states.push_back(root->state);
            for (auto &nd : root->children)
                q.push(nd.first);
        }
        return states;
    }
};
} // namespace librav

#endif /* TRAVERSAL_HPP */
