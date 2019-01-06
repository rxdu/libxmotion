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

#include "graph/tree.hpp"
#include "cvdraw/cvdraw.hpp"

namespace librav
{
namespace RRTViz
{
template <typename TreeType>
void DrawTree(CvCanvas &canvas, TreeType *tree)
{
    for (auto it = tree->vertex_begin(); it != tree->vertex_end(); ++it)
    {
        for (auto child : it->GetNeighbours())
        {
            CPoint pt1(it->state_->values_[0], it->state_->values_[1]);
            CPoint pt2(child->state_->values_[0], child->state_->values_[1]);
            canvas.DrawLine(pt1, pt2);
        }
    }
}

template <typename StateType>
void DrawStraightBranch(CvCanvas &canvas, StateType *start, StateType *end)
{
    CPoint pt1(start->values_[0], start->values_[1]);
    CPoint pt2(end->values_[0], end->values_[1]);
    canvas.DrawLine(pt1, pt2);
}

template <typename StateType>
void DrawStraightPath(CvCanvas &canvas, std::vector<StateType *> path)
{
    for (int i = 0; i < path.size() - 1; ++i)
    {
        auto start = path[i];
        auto end = path[i + 1];
        CPoint pt1(start->values_[0], start->values_[1]);
        CPoint pt2(end->values_[0], end->values_[1]);
        canvas.DrawLine(pt1, pt2, CvColors::blue_color, 2);
    }

    auto start = path.front();
    auto end = path.back();
    CPoint pt1(start->values_[0], start->values_[1]);
    CPoint pt2(end->values_[0], end->values_[1]);
    canvas.DrawPoint(pt1, 3, CvColors::start_color);
    canvas.DrawPoint(pt2, 3, CvColors::finish_color);
}
}; // namespace RRTViz
} // namespace librav

#endif /* RRT_DRAW_HPP */
