/* 
 * kd_tree.hpp
 * 
 * Created on: Dec 31, 2018 10:27
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef KD_TREE_HPP
#define KD_TREE_HPP

#include <limits>
#include <algorithm>

#include "sampling/details/tree/nanoflann.hpp"
#include "sampling/details/base/tree_adapter.hpp"

namespace librav
{
template <typename Space>
class KdTree : public TreeAdapter<Space>
{
  public:
    // using TreeType = Tree<typename Space::StateType *, double>;
    using BaseType = TreeAdapter<Space>;
    using StateType = typename TreeAdapter<Space>::StateType;
    using PathType = typename TreeAdapter<Space>::PathType;

    // inherite constructors
    // using Tree<typename Space::StateType *, double>::Tree;
    using TreeAdapter<Space>::TreeAdapter;

    typedef KDTreeSingleIndexDynamicAdaptor<
        L2_Simple_Adaptor<double, Space>,
        PointCloud<num_t>,
        3 /* dim */
        >
        my_kd_tree_t;

  public:
    void AddTreeNode(StateType *sstate) final
    {
        // TreeType::AddVertex(sstate);
    }

    void ConnectTreeNodes(StateType *sstate, StateType *dstate, double dist) final
    {
        // TreeType::AddEdge(sstate, dstate, dist);
    }

    PathType TraceBackToRoot(StateType *state) final
    {
        // PathType path;
        // path.push_back(state);
        // auto parent = TreeType::GetParentVertex(state);
        // while (parent != TreeType::vertex_end())
        // {
        //     path.push_back(parent->state_);
        //     parent = TreeType::GetParentVertex(parent->vertex_id_);
        // }
        // std::reverse(path.begin(), path.end());
        // return path;
    }

    StateType *FindNearest(StateType *state) final
    {
        // double min_dist = std::numeric_limits<double>::max();
        // StateType *min_state = nullptr;
        // for (auto it = TreeType::vertex_begin(); it != TreeType::vertex_end(); ++it)
        // {
        //     double dist = BaseType::space->EvaluateDistance(state, it->state_);
        //     if (dist < min_dist)
        //     {
        //         min_dist = dist;
        //         min_state = it->state_;
        //     }
        // }
        // return min_state;
    }

    std::vector<StateType *> FindNear(StateType *state) final
    {
    }
};
} // namespace librav

#endif /* KD_TREE_HPP */
