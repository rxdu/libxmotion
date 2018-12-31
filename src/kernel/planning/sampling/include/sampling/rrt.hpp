/* 
 * rrt.hpp
 * 
 * Created on: Dec 30, 2018 06:01
 * Description: RRT algorithm
 * 
 * Reference:
 *  [1] http://msl.cs.illinois.edu/~lavalle/sub/rrt.py
 *  [2] SMP by Sertac Karaman: http://karaman.mit.edu/software.html
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RRT_HPP
#define RRT_HPP

#include <cstdint>
#include <cassert>

#include "sampling/details/base/planner_base.hpp"
#include "sampling/details/tree/basic_tree.hpp"

namespace librav
{
template <typename Space, typename Tree = BasicTree<Space>>
class RRT : public PlannerBase<Space, Tree>
{
  public:
    using BaseType = PlannerBase<Space, Tree>;
    using StateType = typename Space::StateType;

    // inerit base constructor
    using PlannerBase<Space, Tree>::PlannerBase;

  public:
    int Search(StateType *start, StateType *goal, int32_t iter = -1) override
    {
        assert(BaseType::Steer != nullptr);

        int32_t iter_num = 10000;
        if (iter > 0)
            iter_num = iter;

        // add start state to tree
        BaseType::tree_.AddVertex(start);

        // grow tree and look for goal state
        for (int32_t k = 0; k < iter_num; ++k)
        {
            // std::cout << "iteration: " << k << std::endl;

            // 1. Sample a new state from the obstacle-free space
            auto rand_state = BaseType::space_->SampleUniform();
            if (!BaseType::CheckStateValidity(rand_state))
                continue;

            // 2. Find the nearest vertex
            auto nearest = BaseType::tree_.FindNearest(rand_state);
            // std::cout << "nearest id: " << nearest->id_ << std::endl;

            // 3. Extend the nearest vertex towards the sample
            auto new_state = BaseType::Steer(nearest, rand_state);

            // 4. Check the new trajectory for collision
            if (BaseType::CheckPathValidity(nearest, new_state))
            {
                // 5. Add the new collision-free trajectory to the tree
                // TODO do not need to calculate distance twice, should retrieve info from Steer
                BaseType::tree_.AddEdge(nearest, new_state, BaseType::space_->EvaluateDistance(nearest, new_state));
                // std::cout << "added node: " << new_state->id_ << std::endl;

                if (BaseType::CheckGoal(new_state, goal, 1))
                {
                    BaseType::tree_.AddEdge(new_state, goal, BaseType::space_->EvaluateDistance(new_state, goal));

                    std::cout << "path found at iteration " << k << std::endl;
                    auto path = BaseType::tree_.TraceBackToRoot(goal);

                    for (auto &wp : path)
                        std::cout << *wp << std::endl;

                    break;
                }
            }
        }

        // Exit with error
        return 0;
    }
};
} // namespace librav

#endif /* RRT_HPP */
