/* 
 * rrg.hpp
 * 
 * Created on: Dec 30, 2018 06:00
 * Description: RRG algorithm
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RRG_HPP
#define RRG_HPP

#include <cmath>
#include <cstdint>
#include <cassert>

#include "sampling/details/base/planner_base.hpp"
#include "sampling/details/tree/basic_tree.hpp"
#include "sampling/details/tree/kd_tree.hpp"
#include "sampling/details/tree/kd_graph.hpp"

#include "graph/algorithms/dijkstra.hpp"

// #define SHOW_TREE_GROWTH

#ifdef SHOW_TREE_GROWTH
#include "lightviz/details/rrt_draw.hpp"
#endif

namespace librav
{
template <typename Space, typename Tree = KdGraph<Space>>
class RRG : public PlannerBase<Space, Tree>
{
  public:
    using BaseType = PlannerBase<Space, Tree>;
    using StateType = typename Space::StateType;
    using PathType = typename BaseType::PathType;

    // inerit base constructor
    using PlannerBase<Space, Tree>::PlannerBase;

  public:
    void SetOptimizationConstant(double gamma) { gamma_ = gamma; }

    void SetFixedRadius(double radius)
    {
        use_fixed_radius_ = true;
        fixed_radius_ = radius;
    }

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

        std::vector<StateType *> state_to_goal_candidates;

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
                rrtdraw.DrawStraightBranch(nearest, new_state);
                CvDraw::ShowImageFrame(canvas.paint_area, "RRG");
#endif

                // 6. Make connections with vertices that lie within a ball of certain radius
                double radius;
                if (!use_fixed_radius_)
                {
                    double num_vertices = BaseType::tree_.GetTotalTreeNodeNumber();
                    radius = gamma_ * std::pow(std::log(num_vertices) / num_vertices, 1.0 / (static_cast<double>(Space::DimensionSize)));
                    if (radius > BaseType::extend_step_size_)
                        radius = BaseType::extend_step_size_;
                }
                else
                    radius = fixed_radius_;
                auto near_states = BaseType::tree_.FindNear(rand_state, radius);

                for (auto &ns : near_states)
                {
                    if (ns == nearest)
                        continue;

                    if (BaseType::CheckPathValidity(nearest, new_state))
                    {
                        BaseType::tree_.ConnectTreeNodes(ns, new_state, nearest_to_new_dist);
                    }
#ifdef SHOW_TREE_GROWTH
                    rrtdraw.DrawStraightBranch(ns, new_state);
                    CvDraw::ShowImageFrame(canvas.paint_area, "RRG");
#endif
                }

                if (k % 100 == 0)
                    std::cout << "iteration passed: " << k << std::endl;

                // check goal
                if (BaseType::CheckGoal(new_state, goal, BaseType::extend_step_size_))
                {
                    BaseType::tree_.ConnectTreeNodes(new_state, goal, BaseType::space_->EvaluateDistance(new_state, goal));
                    state_to_goal_candidates.push_back(rand_state);

                    std::cout << "candidate path found at iteration " << k << std::endl;

#ifdef SHOW_TREE_GROWTH
                    rrtdraw.DrawStraightBranch(new_state, goal);
                    // rrtdraw.DrawStraightPath(path);
                    CvDraw::ShowImageFrame(canvas.paint_area, "RRG");
#endif
                }
            }
        }

        PathType path;
        if (!state_to_goal_candidates.empty())
        {
            // reconstructing path
            path = Dijkstra::Search(&(BaseType::tree_), start, goal);
#ifdef SHOW_TREE_GROWTH
            rrtdraw.DrawStraightPath(path);
            CvDraw::ShowImageFrame(canvas.paint_area, "RRG", 0);
#endif
            for (auto &wp : path)
                std::cout << *wp << std::endl;
        }
        else
        {
            std::cout << "failed to find a path" << std::endl;
        }

        return path;
    }

  private:
    bool use_fixed_radius_ = false;
    double fixed_radius_ = 0.0;
    double gamma_ = 1.0;
};
} // namespace librav

#endif /* RRG_HPP */
