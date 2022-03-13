/* 
 * rrg.hpp
 * 
 * Created on: Dec 30, 2018 06:00
 * Description: RRG algorithm
 * 
* Reference:
 *  [1] Karaman, S., and E. Frazzoli. 2010. “Incremental Sampling-Based 
 *      Algorithms for Optimal Motion Planning.” Robotics Science and Systems VI.
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RRG_HPP
#define RRG_HPP

#include <cmath>
#include <cstdint>
#include <cassert>

#include "sampling/base/planner_base.hpp"
#include "sampling/tree/kd_graph.hpp"

#include "graph/search/dijkstra.hpp"

// #define SHOW_TREE_GROWTH

#ifdef SHOW_TREE_GROWTH
#include "sampling/rrt_draw.hpp"
// #define SHOW_INTERMEDIATE_STEPS
#endif

namespace robosw
{
template <typename Space>
class RRG : public PlannerBase<Space, KdGraph<Space>>
{
  public:
    using Tree = KdGraph<Space>;
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
        CvCanvas canvas(50);
        canvas.Resize(BaseType::space_->GetBounds()[0].GetLow(), BaseType::space_->GetBounds()[0].GetHigh(),
                      BaseType::space_->GetBounds()[1].GetLow(), BaseType::space_->GetBounds()[1].GetHigh());
#endif

        int32_t iter_num = 10000;
        if (iter > 0)
            iter_num = iter;

        // add start state to tree
        BaseType::tree_.AddTreeRootNode(start);

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
#ifdef SHOW_INTERMEDIATE_STEPS
                RRTViz::DrawStraightBranch(canvas, nearest, new_state);
                CvIO::ShowImageFrame(canvas.GetPaintArea(), "RRG");
#endif
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
                auto near_states = BaseType::tree_.FindNear(new_state, radius);

                for (auto &ns : near_states)
                {
                    if (ns == nearest)
                        continue;

                    if (BaseType::CheckPathValidity(nearest, new_state))
                    {
                        BaseType::tree_.ConnectTreeNodes(ns, new_state, BaseType::space_->EvaluateDistance(ns, new_state));
#ifdef SHOW_TREE_GROWTH
#ifdef SHOW_INTERMEDIATE_STEPS
                        RRTViz::DrawStraightBranch(canvas, ns, new_state);
                        CvIO::ShowImageFrame(canvas.GetPaintArea(), "RRG");
#endif
#endif
                    }
                }

                // check goal
                if (BaseType::CheckGoal(new_state, goal, BaseType::extend_step_size_))
                {
                    BaseType::tree_.ConnectTreeNodes(new_state, goal, BaseType::space_->EvaluateDistance(new_state, goal));
                    state_to_goal_candidates.push_back(new_state);

                    // std::cout << "candidate path found at iteration " << k << std::endl;

#ifdef SHOW_TREE_GROWTH
#ifdef SHOW_INTERMEDIATE_STEPS
                    RRTViz::DrawStraightBranch(canvas, new_state, goal);
                    CvIO::ShowImageFrame(canvas.GetPaintArea(), "RRG");
#endif
#endif
                }
            }

            // BaseType::tree_.PrintTreeInfo();
        }

        PathType path;
        if (!state_to_goal_candidates.empty())
        {
            // reconstructing path
            path = Dijkstra::Search(&(BaseType::tree_), start, goal);
#ifdef SHOW_TREE_GROWTH
            canvas.Clear();
            RRTViz::DrawGraph(canvas, &(BaseType::tree_));
            RRTViz::DrawStraightPath(canvas, path);
            CvIO::ShowImageFrame(canvas.GetPaintArea(), "RRG", 0);
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
} // namespace robosw

#endif /* RRG_HPP */
