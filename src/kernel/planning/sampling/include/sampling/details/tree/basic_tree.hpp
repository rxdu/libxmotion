/* 
 * basic_tree.hpp
 * 
 * Created on: Dec 30, 2018 22:10
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef BASIC_TREE_HPP
#define BASIC_TREE_HPP

#include <limits>
#include <algorithm>

#include "graph/tree.hpp"
#include "sampling/details/base/tree_adapter.hpp"

namespace librav
{
template <typename Space>
class BasicTree : public Tree<typename Space::StateType *, double>, public TreeAdapter<Space>
{
  public:
    using TreeType = Tree<typename Space::StateType *, double>;
    using AdapterType = TreeAdapter<Space>;
    using StateType = typename TreeAdapter<Space>::StateType;
    using PathType = typename TreeAdapter<Space>::PathType;

    // inherit constructors
    using Tree<typename Space::StateType *, double>::Tree;
    using TreeAdapter<Space>::TreeAdapter;

  public:
    void AddTreeRootNode(StateType *sstate) final
    {
        TreeType::AddVertex(sstate);
    }

    void ConnectTreeNodes(StateType *sstate, StateType *dstate, double dist) final
    {
        TreeType::AddEdge(sstate, dstate, dist);
    }

    void DisconnectTreeNodes(StateType *sstate, StateType *dstate) final
    {
        TreeType::RemoveEdge(sstate, dstate);
    }

    std::size_t GetTotalTreeNodeNumber() final
    {
        return TreeType::GetTotalVertexNumber();
    }

    PathType TraceBackToRoot(StateType *state) final
    {
        PathType path;
        path.push_back(state);
        auto parent = TreeType::GetParentVertex(state);
        while (parent != TreeType::vertex_end())
        {
            path.push_back(parent->state_);
            parent = TreeType::GetParentVertex(parent->vertex_id_);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    StateType *FindNearest(StateType *state) final
    {
        double min_dist = std::numeric_limits<double>::max();
        StateType *min_state = nullptr;
        for (auto it = TreeType::vertex_begin(); it != TreeType::vertex_end(); ++it)
        {
            double dist = AdapterType::space->EvaluateDistance(state, it->state_);
            if (dist < min_dist)
            {
                min_dist = dist;
                min_state = it->state_;
            }
        }
        return min_state;
    }

    std::vector<StateType *> FindNear(StateType *state, double radius) final
    {
    }
};
} // namespace librav

#endif /* BASIC_TREE_HPP */
