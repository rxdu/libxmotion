/* 
 * basic_tree.hpp
 * 
 * Created on: Aug 24, 2018 05:00
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef BASIC_TREE_HPP
#define BASIC_TREE_HPP

#include <list>
#include <cassert>
#include <cstdint>
#include <queue>
#include <unordered_map>

#include "tree/tree_node.hpp"

namespace librav
{
template <typename State, typename Transition = double>
class BasicTree
{
  public:
    using NodeType = typename TreeNode<State, Transition>::NodeType;

    BasicTree() = default;
    ~BasicTree() { delete root_; }

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
            src->children.erase(it);
            delete dst;
        }
    }

  private:
    NodeType *root_ = nullptr;
};
} // namespace librav

#endif /* BASIC_TREE_HPP */
