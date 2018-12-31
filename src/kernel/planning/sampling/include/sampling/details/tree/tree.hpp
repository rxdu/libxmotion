/* 
 * tree.hpp
 * 
 * Created on: Jun 22, 2018 15:49
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TREE_HPP
#define TREE_HPP

#include <list>
#include <cassert>
#include <cstdint>
#include <queue>
#include <unordered_map>

namespace librav
{
template <typename State, typename Transition = double>
struct Node
{
    Node() = default;
    Node(State st) : state(st){};
    ~Node()
    {
        for (auto &child : children)
            delete child.first;
    }

    State state;
    Node<State> *parent;
    std::unordered_map<Node<State> *, Transition> children;

    int32_t GetDepth()
    {
        int32_t depth = 0;
        Node<State> *nd = parent;
        while (nd != nullptr)
        {
            nd = nd->parent;
            depth++;
        }
        return depth;
    }
};

template <typename State, typename Transition = double>
class Tree
{
  public:
    using NodeType = Node<State>;
    using StateType = State;

    Tree() = default;
    ~Tree() { delete root_; }

    NodeType *GetRootNode() { return root_; }

    void SetRootNode(NodeType *node)
    {
        root_ = node;
        root_->parent = nullptr;
    }

    void ConnectNodes(NodeType *src, NodeType *dst, Transition trans)
    {
        assert(src != nullptr && dst != nullptr);
        src->children.emplace(std::make_pair(dst, trans));
        dst->parent = src;
    }

    void DisconnectNodes(NodeType *src, NodeType *dst)
    {
        assert(src != nullptr && dst != nullptr);
        auto it = std::find(src->children.begin(), src->children.end(), dst);
        if (it != src->children.end())
        {
            dst->parent = nullptr;
            dst->parent.erase(it);
        }
    }

    void DFSPreOrder(NodeType *node)
    {
        if (node == nullptr)
            return;
        std::cout << node->state << std::endl;
        for (auto &nd : node->children)
            DFSPreOrder(nd.first);
    }

    void DFSPostOrder(NodeType *node)
    {
        if (node == nullptr)
            return;
        for (auto &nd : node->children)
            DFSPreOrder(nd.first);
        std::cout << node->state << std::endl;
    }

    void BFS()
    {
        std::queue<NodeType *> q;
        NodeType *node;
        q.push(root_);
        while (!q.empty())
        {
            node = q.front();
            q.pop();
            std::cout << node->state << std::endl;
            for (auto &nd : node->children)
                q.push(nd.first);
        }
    }

  private:
    NodeType *root_ = nullptr;
};
} // namespace librav

#endif /* TREE_HPP */
