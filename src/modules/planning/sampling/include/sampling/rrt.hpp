/* 
 * rrt.hpp
 * 
 * Created on: Dec 30, 2018 06:01
 * Description: RRT algorithm
 * 
 * Reference:
 *  [1] http://msl.cs.illinois.edu/~lavalle/sub/rrt.py
 *  [2] SMP by Sertac Karaman: http://karaman.mit.edu/software.html
 *  [3] Karaman, S., and E. Frazzoli. 2010. “Incremental Sampling-Based 
 *      Algorithms for Optimal Motion Planning.” Robotics Science and Systems VI.
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RRT_HPP
#define RRT_HPP

#include <cstdint>
#include <cassert>

#include "sampling/base/planner_base.hpp"
#include "sampling/tree/basic_tree.hpp"
#include "sampling/tree/kd_tree.hpp"

// #define SHOW_TREE_GROWTH

#ifdef SHOW_TREE_GROWTH
#include "sampling/rrt_draw.hpp"
#define SHOW_INTERMEDIATE_STEPS
#endif

namespace ivnav
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
        CvCanvas canvas(50);
        canvas.Resize(BaseType::space_->GetBounds()[0].GetLow(), BaseType::space_->GetBounds()[0].GetHigh(),
                           BaseType::space_->GetBounds()[1].GetLow(), BaseType::space_->GetBounds()[1].GetHigh());
#endif

        int32_t iter_num = 10000;
        if (iter > 0)
            iter_num = iter;

        // add start state to tree
        BaseType::tree_.AddTreeRootNode(start);

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
            auto new_state_pair = BaseType::Steer(nearest, rand_state);
            auto new_state = new_state_pair.first;
            auto nearest_to_new_dist = new_state_pair.second;

            // 4. Check the new trajectory for collision
            if (BaseType::CheckPathValidity(nearest, new_state))
            {
                // 5. Add the new collision-free trajectory to the tree
                BaseType::tree_.ConnectTreeNodes(nearest, new_state, nearest_to_new_dist);

#ifdef SHOW_TREE_GROWTH
#ifdef SHOW_INTERMEDIATE_STEPS
                RRTViz::DrawStraightBranch(canvas, nearest, new_state);
                CvIO::ShowImageFrame(canvas.GetPaintArea(), "RRT");
#endif
#endif

                if (BaseType::CheckGoal(new_state, goal, BaseType::extend_step_size_))
                {
                    BaseType::tree_.ConnectTreeNodes(new_state, goal, BaseType::space_->EvaluateDistance(new_state, goal));
                    path = BaseType::tree_.TraceBackToRoot(goal);

                    std::cout << "path found at iteration " << k << std::endl;
                    for (auto &wp : path)
                        std::cout << *wp << std::endl;

#ifdef SHOW_TREE_GROWTH
                    RRTViz::DrawStraightBranch(canvas, new_state, goal);
                    RRTViz::DrawStraightPath(canvas, path);
                    CvIO::ShowImageFrame(canvas.GetPaintArea(), "RRT", 0);
#endif
                    break;
                }
            }
        }

        return path;
    }
};
} // namespace ivnav

#endif /* RRT_HPP */
