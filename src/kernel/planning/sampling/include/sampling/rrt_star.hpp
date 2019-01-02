/* 
 * rrt_star.hpp
 * 
 * Created on: Dec 30, 2018 06:01
 * Description: RRT* algorithm
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RRT_STAR_HPP
#define RRT_STAR_HPP

#include <cstdint>
#include <cassert>

#include "sampling/details/base/planner_base.hpp"
#include "sampling/details/tree/basic_tree.hpp"

// #define SHOW_TREE_GROWTH

#ifdef SHOW_TREE_GROWTH
#include "lightviz/details/rrt_draw.hpp"
#endif

namespace librav
{
template <typename Space, typename Tree = BasicTree<Space>>
class RRTStar : public PlannerBase<Space, Tree>
{
  public:
    using BaseType = PlannerBase<Space, Tree>;
    using StateType = typename Space::StateType;
    using PathType = typename BaseType::PathType;

    // inerit base constructor
    using PlannerBase<Space, Tree>::PlannerBase;

  public:
    PathType Search(StateType *start, StateType *goal, int32_t iter = -1) override
    {
        assert(BaseType::Steer != nullptr);

        int32_t iter_num = 10000;
        if (iter > 0)
            iter_num = iter;

        // add start state to tree
        BaseType::tree_.AddTreeNode(start);

        PathType path;
        std::vector<StateType*> state_to_goal_candidates;

        // grow tree and look for goal state
        for (int32_t k = 0; k < iter_num; ++k)
        {

            // 1. Sample a new state
            auto rand_state = BaseType::space_->SampleUniform();
            if (!BaseType::CheckStateValidity(rand_state))
                continue;

            // 2. Compute the set of all near vertices
            auto near_states = BaseType::tree_.FindNear(rand_state);

            // 3. Find the best parent and extend from that parent
            StateType *min_state_candidate;
            if (near_states.empty())
                // 3.a extend the nearest if near state set is empty
                min_state_candidate = BaseType::tree_.FindNearest(rand_state);
            else
                // 3.b extend the best parent within the near vertices
                min_state_candidate = FindBestParent(new_state, near_states);
            std::pair<StateType *, double> min_state_pair;
            auto min_state_pair = BaseType::Steer(min_state_candidate, rand_state);
            auto min_state = new_state_pair.first;
            auto nearest_to_min_dist = new_state_pair.second;

            // 3.c add the trajectory from the best parent to the tree
            if (BaseType::CheckPathValidity(min_state, new_state))
            {
                BaseType::tree_.ConnectTreeNodes(min_state, new_state, nearest_to_min_dist);

                if (BaseType::CheckGoal(new_state, goal, 1))
                {
                    // BaseType::tree_.ConnectTreeNodes(new_state, goal, BaseType::space_->EvaluateDistance(new_state, goal));
                    // path = BaseType::tree_.TraceBackToRoot(goal);
                    state_to_goal_candidates.push_back(new_state);

                    std::cout << "candidate path found at iteration " << k << std::endl;
                    // for (auto &wp : path)
                    //     std::cout << *wp << std::endl;

                    break;
                }
            }

            // 4. Rewire the tree
            if (!near_states.empty())
                RewireBranches(new_state, near_states);
        }
    }

  private:
    StateType *FindBestParent(StateType *state, const std::vector<StateType *> &near)
    {
    }

    void RewireBranches(StateType *new_state, const std::vector<StateType *> &near)
    {
    }
};
} // namespace librav

#endif /* RRT_STAR_HPP */
