/* 
 * rrt_star.hpp
 * 
 * Created on: Dec 30, 2018 06:01
 * Description: RRT* algorithm
 *
 * Reference:
 *  [1] SMP by Sertac Karaman: http://karaman.mit.edu/software.html
 *  [2] Karaman, S., and E. Frazzoli. 2010. “Incremental Sampling-Based 
 *      Algorithms for Optimal Motion Planning.” Robotics Science and Systems VI.
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RRT_STAR_HPP
#define RRT_STAR_HPP

#include <map>
#include <cstdint>
#include <cassert>

#include "sampling/base/planner_base.hpp"
#include "sampling/tree/kd_tree_motion.hpp"

// #define SHOW_TREE_GROWTH

#ifdef SHOW_TREE_GROWTH
#include "sampling/rrt_draw.hpp"
// #define SHOW_INTERMEDIATE_STEPS
#endif

namespace librav
{
template <typename Space, typename Tree = KdTreeMotion<Space>>
class RRTStar : public PlannerBase<Space, Tree>
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
        std::vector<StateType *> state_to_goal_candidates;

        // grow tree and look for goal state
        for (int32_t k = 0; k < iter_num; ++k)
        {
            // 1. Sample a new state
            auto rand_state = BaseType::space_->SampleUniform();
            if (!BaseType::CheckStateValidity(rand_state))
                continue;

            // 2. Find and extend the nearest vertex towards the sample
            auto nearest = BaseType::tree_.FindNearest(rand_state);

            auto new_state_pair = BaseType::Steer(nearest, rand_state);
            auto new_state = new_state_pair.first;
            auto nearest_to_new_dist = new_state_pair.second;

            if (BaseType::CheckPathValidity(nearest, new_state))
            {
                // 3. Find the best parent vertex for new_state from neare vertices
                auto min_state = nearest;
                auto min_dist = nearest_to_new_dist;

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

                // only check if there is a better min_state if near state set is non-empty
                if (!near_states.empty())
                {
                    auto best_parent_pair = FindBestParent(new_state, near_states, min_state, min_dist);
                    min_state = best_parent_pair.first;
                    min_dist = best_parent_pair.second;
                }

                // 4. Add the path from the best parent to the tree
                BaseType::tree_.ConnectTreeNodes(min_state, new_state, min_dist);

                // 5. Rewire the tree
                if (!near_states.empty())
                    RewireBranches(new_state, min_state, near_states);

                // check if new state is near goal
                if (BaseType::CheckGoal(new_state, goal, BaseType::extend_step_size_))
                {
                    state_to_goal_candidates.push_back(new_state);
                    // std::cout << "candidate path found at iteration " << k << std::endl;
                }

#ifdef SHOW_TREE_GROWTH
#ifdef SHOW_INTERMEDIATE_STEPS
                canvas.Clear();
                RRTViz::DrawTree(canvas, &(BaseType::tree_));
                CvIO::ShowImageFrame(canvas.GetPaintArea(), "RRT*");
#endif
#endif
            }
        }

        if (!state_to_goal_candidates.empty())
        {
            std::cout << "number of candidates: " << state_to_goal_candidates.size() << std::endl;

            std::map<double, StateType *> candidate_map;

            for (auto candidate : state_to_goal_candidates)
            {
                double cost_to_go = BaseType::space_->EvaluateDistance(candidate, goal);
                double total_cost = BaseType::tree_.GetStateCost(candidate) + cost_to_go;
                candidate_map.insert(std::make_pair(total_cost, candidate));
            }

            auto best_candidate = candidate_map.begin()->second;
            BaseType::tree_.ConnectTreeNodes(best_candidate, goal, BaseType::space_->EvaluateDistance(best_candidate, goal));
            path = BaseType::tree_.TraceBackToRoot(goal);

            for (auto &wp : path)
                std::cout << *wp << std::endl;
        }

#ifdef SHOW_TREE_GROWTH
        canvas.Clear();
        RRTViz::DrawTree(canvas, &(BaseType::tree_));
        RRTViz::DrawStraightPath(canvas, path);
        CvIO::ShowImageFrame(canvas.GetPaintArea(), "RRT*", 0);
#endif

        return path;
    }

  private:
    bool use_fixed_radius_ = false;
    double fixed_radius_ = 0.0;
    double gamma_ = 1.0;

    std::pair<StateType *, double> FindBestParent(StateType *new_state, const std::vector<StateType *> &near, StateType *min_state, double min_dist)
    {
        StateType *output_min_state = min_state;
        double output_min_dist = min_dist;

        double cost_new = BaseType::tree_.GetStateCost(new_state);
        for (auto near_state : near)
        {
            if (BaseType::CheckPathValidity(near_state, new_state))
            {
                double edge_cost = BaseType::space_->EvaluateDistance(near_state, new_state);
                double combined_cost = BaseType::tree_.GetStateCost(near_state) + edge_cost;
                if (combined_cost < cost_new)
                {
                    cost_new = combined_cost;
                    output_min_state = near_state;
                    output_min_dist = edge_cost;
                }
            }
        }
        return std::make_pair(output_min_state, output_min_dist);
    }

    void RewireBranches(StateType *new_state, StateType *min_state, const std::vector<StateType *> &near)
    {
        for (auto near_state : near)
        {
            if (near_state == min_state)
                continue;

            if (BaseType::CheckPathValidity(new_state, near_state))
            {
                double cost_near = BaseType::tree_.GetStateCost(near_state);
                double new_dist = BaseType::space_->EvaluateDistance(new_state, near_state);
                double combined_cost = BaseType::tree_.GetStateCost(new_state) + new_dist;
                if (cost_near > combined_cost)
                    BaseType::tree_.ReconnectTreeNodes(new_state, near_state, new_dist);
            }
        }
    }
};
} // namespace librav

#endif /* RRT_STAR_HPP */
