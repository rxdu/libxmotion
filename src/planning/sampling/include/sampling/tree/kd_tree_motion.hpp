/* 
 * kd_tree_motion.hpp
 * 
 * Created on: Jan 03, 2019 10:23
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef KD_TREE_MOTION_HPP
#define KD_TREE_MOTION_HPP

#include <limits>
#include <algorithm>
#include <unordered_map>

#include "spatial/spatial.hpp"
#include "spatial/point_multiset.hpp"
#include "spatial/neighbor_iterator.hpp"

#include "graph/tree.hpp"
#include "sampling/base/tree_adapter.hpp"

namespace robosw
{
template <typename Space>
class KdTreeMotion : public Tree<typename Space::StateType *, double>, public TreeAdapter<Space>
{
  public:
    using TreeType = Tree<typename Space::StateType *, double>;
    using AdapterType = TreeAdapter<Space>;
    using StateType = typename TreeAdapter<Space>::StateType;
    using PathType = typename TreeAdapter<Space>::PathType;

    // inherit constructors
    using Tree<typename Space::StateType *, double>::Tree;
    using TreeAdapter<Space>::TreeAdapter;

    // setup Kd-tree types
    struct StateAccessor
    {
        double operator()(spatial::dimension_type dim, StateType *state) const
        {
            return (*state)[dim];
        }
    };

    using KdTreeType = spatial::point_multiset<Space::DimensionSize,
                                               StateType *,
                                               spatial::accessor_less<StateAccessor, StateType *>>;

  public:
    void AddTreeRootNode(StateType *sstate) final
    {
        TreeType::AddVertex(sstate);
        kdtree_.insert(sstate);
        g_cost_[sstate->id_] = 0.0;
    }

    void ConnectTreeNodes(StateType *sstate, StateType *dstate, double dist) final
    {
        TreeType::AddEdge(sstate, dstate, dist);
        kdtree_.insert(sstate);
        kdtree_.insert(dstate);
        g_cost_[dstate->id_] = g_cost_[sstate->id_] + dist;
    }

    void DisconnectTreeNodes(StateType *sstate, StateType *dstate) final
    {
        TreeType::RemoveEdge(sstate, dstate);
        g_cost_[dstate->id_] = std::numeric_limits<double>::max();
    }

    void ReconnectTreeNodes(StateType *new_sstate, StateType *dstate, double dist)
    {
        StateType *sstate = TreeType::GetParentVertex(dstate)->state_;
        TreeType::RemoveEdge(sstate, dstate);
        TreeType::AddEdge(new_sstate, dstate, dist);

        // update costs
        double old_cost = g_cost_[dstate->id_];
        g_cost_[dstate->id_] = g_cost_[new_sstate->id_] + dist;
        double cost_err = old_cost - g_cost_[dstate->id_];
        auto children = TreeType::FindVertex(dstate)->GetNeighbours();
        while (!children.empty())
        {
            std::vector<typename TreeType::vertex_iterator> childset;
            for (auto &child : children)
            {
                g_cost_[child->state_->id_] -= cost_err;
                auto childs_children = child->GetNeighbours();
                if (!childs_children.empty())
                    childset.insert(childset.end(), childs_children.begin(), childs_children.end());
            }
            children = childset;
        }
    }

    void SetStateCost(StateType *state, double cost)
    {
        g_cost_[state->id_] = cost;
    }

    double GetStateCost(StateType *state)
    {
        return g_cost_[state->id_];
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
        int i = 0;
        while (parent != TreeType::vertex_end())
        {
            path.push_back(parent->state_);
            std::cout << "backtracking " << i++ << std::endl;
            parent = TreeType::GetParentVertex(parent->vertex_id_);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    StateType *FindNearest(StateType *state) final
    {
        spatial::neighbor_iterator<KdTreeType> iter = spatial::neighbor_begin(kdtree_, state);
        return (*iter);
    }

    std::vector<StateType *> FindNear(StateType *state, double radius) final
    {
        std::vector<StateType *> near_states;

        spatial::neighbor_iterator<KdTreeType> iter = spatial::neighbor_begin(kdtree_, state);

        for (auto it = iter; it != kdtree_.end(); ++it)
        {
            if (distance(it) <= radius)
                near_states.push_back(*it);
            else
                break;
        }

        return near_states;
    }

  private:
    KdTreeType kdtree_;
    std::unordered_map<int64_t, double> g_cost_;
};
} // namespace robosw

#endif /* KD_TREE_MOTION_HPP */
