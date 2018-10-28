/* 
 * unique_tree.hpp
 * 
 * Created on: Aug 24, 2018 05:36
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef UNIQUE_TREE_HPP
#define UNIQUE_TREE_HPP

#include <list>
#include <cassert>
#include <cstdint>
#include <queue>
#include <algorithm>

#include "tree/tree_node.hpp"
#include "tree/algorithms/search.hpp"

namespace librav
{
template <typename State, typename Transition = double>
class UniqueTree
{
  public:
    using NodeType = typename TreeNode<State, Transition>::NodeType;
    using StateType = State;
    using TransitionType = Transition;

    UniqueTree() = default;
    ~UniqueTree() { delete root_; }

    UniqueTree(const UniqueTree<State, Transition> &other)
    {
        NodeType *node = other.root_;

        std::queue<NodeType *> q;
        q.push(node);
        while (!q.empty())
        {
            node = q.front();
            q.pop();
            for (auto &nd : node->children)
            {
                this->ConnectNodes(node->state, nd.first->state, nd.second);
                q.push(nd.first);
            }
        }
    }

    UniqueTree<State, Transition> &operator=(const UniqueTree<State, Transition> &other)
    {
        UniqueTree temp = other;
        std::swap(*this, temp);
        return *this;
    }

    UniqueTree(UniqueTree<State, Transition> &&other)
    {
        root_ = other.root_;
        other.root_ = nullptr;
    }

    UniqueTree<State, Transition> &operator=(UniqueTree<State, Transition> &&other)
    {
        std::swap(this->root_, other.root_);
        return *this;
    }

    NodeType *GetRootNode() { return root_; }

    void SetRootNode(NodeType *node)
    {
        root_ = node;
        root_->parent = nullptr;
    }

    void ConnectNodes(State src, State dst, Transition trans = 0.0)
    {
        // set root if this is the very first connection in the tree
        if (root_ == nullptr)
        {
            NodeType *new_src = new NodeType(src);
            NodeType *new_dst = new NodeType(dst);

            new_dst->parent = new_src;
            new_src->children.emplace(std::make_pair(new_dst, trans));

            SetRootNode(new_src);
        }
        else
        {
            NodeType *src_nd = Search::BFS(this, src);
            NodeType *dst_nd = Search::BFS(this, dst);

            if (src_nd == nullptr)
            {
                std::cerr << "non-existing src node in the tree" << std::endl;
                return;
            }

            if (dst_nd != nullptr)
            {
                if (dst_nd->parent != src_nd)
                {
                    std::cerr << "conflicting connection between src and dst nodes in the tree" << std::endl;
                    std::cerr << " -- src: " << src_nd->state << std::endl;
                    std::cerr << " -- dst: " << dst_nd->state << std::endl;
                }

                return;
            }

            // Create new connection when: src_nd != nullptr && dst_nd == nullptr
            NodeType *new_dst = new NodeType(dst);
            new_dst->parent = src_nd;
            src_nd->children.emplace(std::make_pair(new_dst, trans));
        }
    }

    void DisconnectNodes(State src, State dst)
    {
        // return if tree is empty
        if (root_ == nullptr)
            return;

        NodeType *src_nd = Search::BFS(this, src);
        NodeType *dst_nd = Search::BFS(this, dst);

        if (src_nd == nullptr || dst_nd == nullptr)
        {
            std::cerr << "non-existing src or dst node in the tree" << std::endl;
            return;
        }

        auto it = src_nd->children.find(dst_nd);
        if (it != src_nd->children.end())
        {
            dst_nd->parent = nullptr;
            src_nd->children.erase(it);
            delete dst_nd;
        }
    }

  private:
    NodeType *root_ = nullptr;
};
} // namespace librav

#endif /* UNIQUE_TREE_HPP */
