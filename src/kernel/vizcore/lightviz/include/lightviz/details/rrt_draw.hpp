/* 
 * rrt_draw.hpp
 * 
 * Created on: Dec 31, 2018 08:18
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RRT_DRAW_HPP
#define RRT_DRAW_HPP

#include <vector>
#include <cstdint>

#include "canvas/cv_draw.hpp"
#include "graph/tree.hpp"

#include "lightviz/details/cartesian_canvas.hpp"

namespace librav
{
class RRTDraw
{
  public:
    RRTDraw(CartesianCanvas &canvas) : canvas_(canvas){};

    template <typename TreeType>
    void DrawTree(TreeType *tree)
    {
        for (auto it = tree->GetInternalTree()->vertex_begin(); it != tree->GetInternalTree()->vertex_end(); ++it)
        {
            for (auto child : it->GetNeighbours())
            {
                auto pt1 = canvas_.ConvertCartisianToPixel(it->state_->values_[0], it->state_->values_[1]);
                auto pt2 = canvas_.ConvertCartisianToPixel(child->state_->values_[0], child->state_->values_[1]);
                CvDraw::DrawLine(canvas_.paint_area, cv::Point(pt1.x, pt1.y), cv::Point(pt2.x, pt2.y));
            }
        }
    }

    template <typename StateType>
    void DrawStraightBranch(StateType *start, StateType *end)
    {
        auto pt1 = canvas_.ConvertCartisianToPixel(start->values_[0], start->values_[1]);
        auto pt2 = canvas_.ConvertCartisianToPixel(end->values_[0], end->values_[1]);
        CvDraw::DrawLine(canvas_.paint_area, cv::Point(pt1.x, pt1.y), cv::Point(pt2.x, pt2.y));
    }

    template <typename StateType>
    void DrawTreeStraightPath(std::vector<StateType *> path)
    {
        // for (auto it = tree->GetInternalTree()->vertex_begin(); it != tree->GetInternalTree()->vertex_end(); ++it)
        // for (auto &state : path)
        // {
        //     for (auto child : it->GetNeighbours())
        //     {
        //         auto pt1 = canvas_.ConvertCartisianToPixel(it->state_->values_[0], it->state_->values_[1]);
        //         auto pt2 = canvas_.ConvertCartisianToPixel(child->state_->values_[0], child->state_->values_[1]);
        //         CvDraw::DrawLine(canvas_.paint_area, cv::Point(pt1.x, pt1.y), cv::Point(pt2.x, pt2.y));
        //     }
        // }
    }

  private:
    CartesianCanvas &canvas_;
};
} // namespace librav

#endif /* RRT_DRAW_HPP */
