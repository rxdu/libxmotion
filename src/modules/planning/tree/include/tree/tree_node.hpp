/* 
 * tree_node.hpp
 * 
 * Created on: Aug 24, 2018 00:15
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TREE_NODE_HPP
#define TREE_NODE_HPP

#ifdef USE_UNORDERED_MAP
// unordered_map organizes TreeNode with hashing by address
#include <unordered_map>
#else
// map supports sorting according to TreeNode address
#include <map>
#endif

namespace librav
{
/// Planning tree node
template <typename State, typename Transition = double>
struct TreeNode
{
    using NodeType = TreeNode<State, Transition>;

    TreeNode() = default;
    TreeNode(State st) : state(st){};
    ~TreeNode()
    {
        for (auto &child : children)
            delete child.first;
    }

    State state;

    NodeType *parent;
#ifdef USE_UNORDERED_MAP
    std::unordered_map<NodeType *, Transition> children;
#else
    std::map<NodeType *, Transition> children;
#endif
    int32_t GetDepth()
    {
        int32_t depth = 0;
        NodeType *nd = parent;
        while (nd != nullptr)
        {
            nd = nd->parent;
            depth++;
        }
        return depth;
    }
};
} // namespace librav

#endif /* TREE_NODE_HPP */
