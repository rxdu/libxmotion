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

namespace xmotion {
namespace RRTViz {
template <typename TreeType>
void DrawTree(CvCanvas &canvas, TreeType *tree) {
  for (auto it = tree->vertex_begin(); it != tree->vertex_end(); ++it) {
    for (auto child : it->GetNeighbours()) {
      CPoint pt1(it->state->values_[0], it->state->values_[1]);
      CPoint pt2(child->state->values_[0], child->state->values_[1]);
      canvas.DrawLine(pt1, pt2, CvColors::silver_color);
      canvas.DrawPoint(pt2, 2, CvColors::green_color);
    }
  }
}

template <typename GraphType>
void DrawGraph(CvCanvas &canvas, GraphType *graph) {
  // draw all vertices
  for (auto vertex = graph->vertex_begin(); vertex != graph->vertex_end();
       ++vertex) {
    // current vertex center coordinate
    int32_t loc_x = vertex->state->values_[0];
    int32_t loc_y = vertex->state->values_[1];
    CPoint center(loc_x, loc_y);
    canvas.DrawPoint(center, 2, CvColors::green_color);
  }

  // draw all edges
  auto edges = graph->GetAllEdges();
  for (auto &edge_it : edges) {
    auto edge = *edge_it;
    CPoint pt1(edge.src->state->values_[0], edge.src->state->values_[1]);
    CPoint pt2(edge.dst->state->values_[0], edge.dst->state->values_[1]);
    // canvas.DrawLine(pt1, pt2, cv::Scalar(237, 149, 100));
    canvas.DrawLine(pt1, pt2, CvColors::silver_color);
  }
}

template <typename StateType>
void DrawStraightBranch(CvCanvas &canvas, std::shared_ptr<StateType> start,
                        std::shared_ptr<StateType> end) {
  CPoint pt1(start->values_[0], start->values_[1]);
  CPoint pt2(end->values_[0], end->values_[1]);
  canvas.DrawLine(pt1, pt2, CvColors::silver_color);
  canvas.DrawPoint(pt2, 2, CvColors::green_color);
}

template <typename StateType>
void DrawStraightPath(CvCanvas &canvas,
                      std::vector<std::shared_ptr<StateType>> path) {
  for (int i = 0; i < path.size() - 1; ++i) {
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
  canvas.DrawPoint(pt1, 5, CvColors::start_color);
  canvas.DrawPoint(pt2, 5, CvColors::finish_color);
}
};  // namespace RRTViz
}  // namespace xmotion

#endif /* RRT_DRAW_HPP */
