/* 
 * tree_adapter.hpp
 * 
 * Created on: Dec 30, 2018 21:38
 * Description: a tree for sampling-based planning should satisfy
 *       the following conditions:
 *      1. The "Space" type should be derived from SpaceBase
 *      2. Provide implementations to all pure virual functions
 * 
 *      See "sampling/base/tree/basic_tree.hpp" for an example
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TREE_ADAPTER_HPP
#define TREE_ADAPTER_HPP

#include <vector>
#include <type_traits>

namespace rnav
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

    // tree is built inside a given space
    Space *space;

    /****************** To Be Implemented ******************/
    // common interface for tree
    virtual void AddTreeRootNode(StateType *sstate) = 0;
    virtual void ConnectTreeNodes(StateType *sstate, StateType *dstate, double dist) = 0;
    virtual void DisconnectTreeNodes(StateType *sstate, StateType *dstate) = 0;

    virtual std::size_t GetTotalTreeNodeNumber() = 0;
    virtual PathType TraceBackToRoot(StateType *state) = 0;

    virtual StateType *FindNearest(StateType *state) = 0;
    virtual std::vector<StateType *> FindNear(StateType *state, double radius) = 0;
    /*******************************************************/
};
} // namespace rnav

#endif /* TREE_ADAPTER_HPP */
