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
class BasicTree : public TreeAdapter<Space>
{
  public:
    using BaseType = TreeAdapter<Space>;
    using StateType = typename TreeAdapter<Space>::StateType;
    using PathType = typename TreeAdapter<Space>::PathType;
    using TreeAdapter<Space>::TreeAdapter;

  public:
    void AddVertex(StateType *sstate) final
    {
        tree_.AddVertex(sstate);
    }

    void AddEdge(StateType *sstate, StateType *dstate, double dist) final
    {
        tree_.AddEdge(sstate, dstate, dist);
    }

    PathType TraceBackToRoot(StateType *state) final
    {
        PathType path;
        path.push_back(state);
        auto parent = tree_.GetParentVertex(state);
        while (parent != tree_.vertex_end())
        {
            path.push_back(parent->state_);
            parent = tree_.GetParentVertex(parent->vertex_id_);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    StateType *FindNearest(StateType *state) final
    {
        double min_dist = std::numeric_limits<double>::max();
        StateType *min_state = nullptr;
        for (auto it = tree_.vertex_begin(); it != tree_.vertex_end(); ++it)
        {
            double dist = BaseType::space->EvaluateDistance(state, it->state_);
            if (dist < min_dist)
            {
                min_dist = dist;
                min_state = it->state_;
            }
        }
        return min_state;
    }

    std::vector<StateType *> FindNear(StateType *state) final
    {
    }

  private:
    Tree<StateType *, double> tree_;
};
} // namespace librav

#endif /* BASIC_TREE_HPP */
