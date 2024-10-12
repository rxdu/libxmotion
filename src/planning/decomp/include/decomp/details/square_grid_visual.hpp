/*
 * square_grid_visual.hpp
 *
 * Created on: Apr 09, 2018 23:13
 * Description: this file defines a set of grid drawing
 *          primitives that can be used in grid_viz.hpp
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SQUARE_GRID_VISUAL_HPP
#define SQUARE_GRID_VISUAL_HPP

#include <cstdint>

#include "decomp/square_grid.hpp"
#include "graph/graph.hpp"

#ifdef ENABLE_VISUAL
#include "cvdraw/cvdraw.hpp"
#endif

namespace xmotion {
template <typename GridType>
quickviz::CvCanvas CreateCanvas(const GridType &grid, int32_t ppu = 100) {
  int32_t vis_side_size = grid.GetCellSize() * ppu;
  quickviz::CvCanvas canvas(ppu);
  // reserve additional space outside the border
  canvas.Resize(
      -0.5 * grid.GetCellSize(), (grid.SizeX() + 0.5) * grid.GetCellSize(),
      -0.5 * grid.GetCellSize(), (grid.SizeY() + 0.5) * grid.GetCellSize());
  canvas.SetMode(quickviz::CvCanvas::DrawMode::Geometry);
  return canvas;
}

template <typename CellType>
void FillGridCell(quickviz::CvCanvas &canvas, CellType *cell,
                  const cv::Scalar &color) {
  auto pt1 = canvas.ConvertGeometryPointToPixel(cell->vertices[0].x,
                                                cell->vertices[0].y);
  auto pt2 = canvas.ConvertGeometryPointToPixel(cell->vertices[3].x,
                                                cell->vertices[3].y);
  cv::Range rngx(pt1.x, pt2.x);
  cv::Range rngy(pt1.y, pt2.y);
  canvas.GetPaintArea()(rngy, rngx) = color;
}

template <typename GridType>
void DrawGridNet(quickviz::CvCanvas &canvas, const GridType &grid) {
  // draw horizontal lines
  for (int32_t y = 0; y < grid.SizeY(); ++y) {
    auto first_vertex = grid.GetCell(0, y)->vertices[0];
    auto last_vertex = grid.GetCell(grid.SizeX() - 1, y)->vertices[1];
    quickviz::CPoint pt1(first_vertex.x, first_vertex.y);
    quickviz::CPoint pt2(last_vertex.x, last_vertex.y);
    canvas.DrawLine(pt1, pt2, quickviz::CvColors::default_ln_color, 1,
                    cv::LINE_8);
  }
  {
    auto first_vertex = grid.GetCell(0, grid.SizeY() - 1)->vertices[2];
    auto last_vertex =
        grid.GetCell(grid.SizeX() - 1, grid.SizeY() - 1)->vertices[3];
    quickviz::CPoint pt1(first_vertex.x, first_vertex.y);
    quickviz::CPoint pt2(last_vertex.x, last_vertex.y);
    canvas.DrawLine(pt1, pt2, quickviz::CvColors::default_ln_color, 1,
                    cv::LINE_8);
  }

  // draw vertical lines
  for (int32_t x = 0; x < grid.SizeX(); ++x) {
    auto first_vertex = grid.GetCell(x, 0)->vertices[0];
    auto last_vertex = grid.GetCell(x, grid.SizeY() - 1)->vertices[2];
    quickviz::CPoint pt1(first_vertex.x, first_vertex.y);
    quickviz::CPoint pt2(last_vertex.x, last_vertex.y);
    canvas.DrawLine(pt1, pt2, quickviz::CvColors::default_ln_color, 1,
                    cv::LINE_8);
  }
  {
    auto first_vertex = grid.GetCell(grid.SizeX() - 1, 0)->vertices[1];
    auto last_vertex =
        grid.GetCell(grid.SizeX() - 1, grid.SizeY() - 1)->vertices[3];
    quickviz::CPoint pt1(first_vertex.x, first_vertex.y);
    quickviz::CPoint pt2(last_vertex.x, last_vertex.y);
    canvas.DrawLine(pt1, pt2, quickviz::CvColors::default_ln_color, 1,
                    cv::LINE_8);
  }
}

template <typename GridType>
void DrawGridCell(quickviz::CvCanvas &canvas, const GridType &grid) {
  // draw cells
  for (int32_t y = 0; y < grid.SizeY(); ++y)
    for (int32_t x = 0; x < grid.SizeX(); ++x) {
      auto cell = grid.GetCell(x, y);
      if (cell->label == SquareCellLabel::OCCUPIED)
        FillGridCell(canvas, cell, quickviz::CvColors::obs_color);
    }
}

template <typename GridCellType>
void DrawGridPath(quickviz::CvCanvas &canvas,
                  const std::vector<GridCellType *> &path) {
  for (int i = 0; i < path.size() - 1; ++i) {
    auto first_cell = path[i]->center;
    auto next_cell = path[i + 1]->center;
    quickviz::CPoint pt1(first_cell.x, first_cell.y);
    quickviz::CPoint pt2(next_cell.x, next_cell.y);
    canvas.DrawLine(pt1, pt2, quickviz::CvColors::intermediate_color, 1,
                    cv::LINE_8);
  }
}

template <typename GridCellType>
void DrawGridPathStartGoal(quickviz::CvCanvas &canvas,
                           const std::vector<GridCellType *> &path) {
  FillGridCell(canvas, path.front(), quickviz::CvColors::start_color);
  FillGridCell(canvas, path.back(), quickviz::CvColors::finish_color);
}

template <typename GridType>
void DrawGridCost(quickviz::CvCanvas &canvas, const GridType &grid) {
  // draw cells
  for (int32_t y = 0; y < grid.SizeY(); ++y)
    for (int32_t x = 0; x < grid.SizeX(); ++x) {
      auto cell = grid.GetCell(x, y);
      if (cell->label != SquareCellLabel::OCCUPIED)
        FillGridCell(canvas, cell,
                     quickviz::JetColorMap::Transform(cell->cost_map));
    }
}

template <typename GridType>
void DrawGridCostOnly(quickviz::CvCanvas &canvas, const GridType &grid) {
  // draw cells
  for (int32_t y = 0; y < grid.SizeY(); ++y)
    for (int32_t x = 0; x < grid.SizeX(); ++x) {
      auto cell = grid.GetCell(x, y);
      if (cell->label != SquareCellLabel::OCCUPIED) {
        if (cell->cost_map != 0.0)
          FillGridCell(canvas, cell,
                       quickviz::JetColorMap::Transform(cell->cost_map));
      }
    }
}
}  // namespace xmotion

#endif /* SQUARE_GRID_VISUAL_HPP */
