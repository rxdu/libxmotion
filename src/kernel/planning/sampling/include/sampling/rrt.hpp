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

// #define SHOW_TREE_GROWTH

#ifdef SHOW_TREE_GROWTH
#include "lightviz/details/rrt_draw.hpp"
#endif

namespace librav
{
template <typename Space, typename Tree = BasicTree<Space>>
class RRT : public PlannerBase<Space, Tree>
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

#ifdef SHOW_TREE_GROWTH
        CartesianCanvas canvas(50);
        canvas.SetupCanvas(BaseType::space_->GetBounds()[0].GetLow(), BaseType::space_->GetBounds()[0].GetHigh(),
                           BaseType::space_->GetBounds()[1].GetLow(), BaseType::space_->GetBounds()[1].GetHigh());
        RRTDraw rrtdraw(canvas);
#endif

        int32_t iter_num = 10000;
        if (iter > 0)
            iter_num = iter;

        // add start state to tree
        BaseType::tree_.AddTreeNode(start);

        PathType path;
        // grow tree and look for goal state
        for (int32_t k = 0; k < iter_num; ++k)
        {
            // 1. Sample a new state from the obstacle-free space
            auto rand_state = BaseType::space_->SampleUniform();
            if (!BaseType::CheckStateValidity(rand_state))
                continue;

            // 2. Find the nearest vertex
            auto nearest = BaseType::tree_.FindNearest(rand_state);

            // 3. Extend the nearest vertex towards the sample
            auto new_state = BaseType::Steer(nearest, rand_state);

            // 4. Check the new trajectory for collision
            if (BaseType::CheckPathValidity(nearest, new_state))
            {
                // 5. Add the new collision-free trajectory to the tree
                // TODO do not need to calculate distance twice, should retrieve info from Steer
                BaseType::tree_.ConnectTreeNodes(nearest, new_state, BaseType::space_->EvaluateDistance(nearest, new_state));

#ifdef SHOW_TREE_GROWTH
                // rrtdraw.DrawTree(&(BaseType::tree_));
                rrtdraw.DrawStraightBranch(nearest, new_state);
                CvDraw::ShowImageFrame(canvas.paint_area, "RRT");
#endif

                if (BaseType::CheckGoal(new_state, goal, 1))
                {
                    BaseType::tree_.ConnectTreeNodes(new_state, goal, BaseType::space_->EvaluateDistance(new_state, goal));

                    std::cout << "path found at iteration " << k << std::endl;
                    path = BaseType::tree_.TraceBackToRoot(goal);

                    for (auto &wp : path)
                        std::cout << *wp << std::endl;

#ifdef SHOW_TREE_GROWTH
                    rrtdraw.DrawStraightBranch(new_state, goal);
                    rrtdraw.DrawStraightPath(path);
                    CvDraw::ShowImageFrame(canvas.paint_area, "RRT", 0);
#endif
                    break;
                }
            }
        }

        return path;
    }
};
} // namespace librav

#endif /* RRT_HPP */
