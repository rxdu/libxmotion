/* 
 * tree_adapter.hpp
 * 
 * Created on: Dec 30, 2018 21:38
 * Description: a tree used for sampling-based planning should 
 *      satisfy the following conditions:
 *      1. The Tree is built within a Space, which is derived from SpaceBase
 *      2. Basic operations: AddEdge(), FindNearest(), FindNear() 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TREE_ADAPTER_HPP
#define TREE_ADAPTER_HPP

#include <vector>
#include <type_traits>

namespace librav
{
class SpaceBase;

template <typename Space>
struct TreeAdapter
{
    using StateType = typename Space::StateType;
    using PathType = std::vector<StateType *>;

    /****************** type sanity check ******************/
    // check if Space is derived from SpaceBase
    static_assert(std::is_base_of<SpaceBase, Space>::value,
                  "Space should inherit from SpaceBase to ensure interface compatibility");
    /*******************************************************/

    TreeAdapter(Space *s) : space(s) {}

    Space *space;

    // common interface for tree
    virtual void AddTreeNode(StateType *sstate) = 0;
    virtual void ConnectTreeNodes(StateType *sstate, StateType *dstate, double dist) = 0;
    virtual PathType TraceBackToRoot(StateType *state) = 0;

    virtual StateType *FindNearest(StateType *state) = 0;
    virtual std::vector<StateType *> FindNear(StateType *state) = 0;
};
} // namespace librav

#endif /* TREE_ADAPTER_HPP */
