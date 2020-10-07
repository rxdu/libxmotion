/* 
 * curvilinear_grid_draw.hpp
 * 
 * Created on: Nov 01, 2018 10:19
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CURVILINEAR_GRID_DRAW_HPP
#define CURVILINEAR_GRID_DRAW_HPP

#include <cstdint>
#include <functional>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "decomp/curvilinear_grid.hpp"
#include "geometry/geometry_draw.hpp"

namespace autodrive {
namespace CurvilinearGridViz {
// geometric grid
template <typename GridType>
void DrawCurvilinearGrid(CvCanvas &canvas, const GridType &grid, cv::Scalar ln_color = CvColors::lime_color, int32_t thickness = 1)
{
    // draw normal lines
    CPoint spt1(grid.grid_tiles_.front().back()->vertices[2].position.x, grid.grid_tiles_.front().back()->vertices[2].position.y);
    CPoint spt2(grid.grid_tiles_.front().front()->vertices[3].position.x, grid.grid_tiles_.front().front()->vertices[3].position.y);
    canvas.DrawLine(spt1, spt2, ln_color, thickness);
    for (auto &row : grid.grid_tiles_)
    {
        CPoint pt1(row.back()->vertices[0].position.x, row.back()->vertices[0].position.y);
        CPoint pt2(row.front()->vertices[1].position.x, row.front()->vertices[1].position.y);
        canvas.DrawLine(pt1, pt2, ln_color, thickness);
    }

    // draw tangential lines
    for (auto &row : grid.grid_tiles_)
    {
        for (auto &cell : row)
        {
            CPoint pt1(cell->vertices[1].position.x, cell->vertices[1].position.y);
            CPoint pt2(cell->vertices[3].position.x, cell->vertices[3].position.y);
            canvas.DrawLine(pt1, pt2, ln_color, thickness);
        }
        CPoint fpt1(row.back()->vertices[0].position.x, row.back()->vertices[0].position.y);
        CPoint fpt2(row.back()->vertices[2].position.x, row.back()->vertices[2].position.y);
        canvas.DrawLine(fpt1, fpt2, ln_color, thickness);
    }
}

template <typename GridType>
void FillCurvilinearGrid(CvCanvas &canvas, const GridType &grid, cv::Scalar fill_color = CvColors::lime_color, int32_t thickness = 1)
{
    Polygon polygon;

    // add points on the right bound
    for (auto &tile_row : grid.grid_tiles_)
    {
        auto tile = tile_row.front();
        polygon.AddPoint(tile->vertices[3].position.x, tile->vertices[3].position.y);
    }
    auto last_right_tile = grid.grid_tiles_.back().front();
    polygon.AddPoint(last_right_tile->vertices[1].position.x, last_right_tile->vertices[1].position.y);

    // add points on the left bound
    for (auto it = grid.grid_tiles_.rbegin(); it != grid.grid_tiles_.rend(); ++it)
    {
        auto tile_row = *it;
        auto tile = tile_row.back();
        polygon.AddPoint(tile->vertices[0].position.x, tile->vertices[0].position.y);
    }
    auto last_left_tile = grid.grid_tiles_.front().back();
    polygon.AddPoint(last_left_tile->vertices[2].position.x, last_left_tile->vertices[2].position.y);

    GeometryViz::FillPolygon(canvas, polygon, false, fill_color);
}

template <typename GridType>
void DrawCurvilinearGridCost(CvCanvas &canvas, const GridType &grid, cv::Scalar ln_color = CvColors::lime_color, int32_t thickness = 1)
{
    for (auto &tile_row : grid.grid_tiles_)
    {
        for (auto tile : tile_row)
        {
            Polygon polygon;
            polygon.AddPoint(tile->vertices[0].position.x, tile->vertices[0].position.y);
            polygon.AddPoint(tile->vertices[1].position.x, tile->vertices[1].position.y);
            polygon.AddPoint(tile->vertices[3].position.x, tile->vertices[3].position.y);
            polygon.AddPoint(tile->vertices[2].position.x, tile->vertices[2].position.y);
            GeometryViz::FillPolygon(canvas, polygon, false, JetColorMap::Transform(tile->cost_map));
        }
    }

    CurvilinearGridViz::DrawCurvilinearGrid(canvas, grid);
}

template <typename GridType>
void DrawCurvilinearGridCostOnly(CvCanvas &canvas, const GridType &grid, cv::Scalar ln_color = CvColors::lime_color, int32_t thickness = 1)
{
    for (auto &tile_row : grid.grid_tiles_)
    {
        for (auto tile : tile_row)
        {
            Polygon polygon;
            polygon.AddPoint(tile->vertices[0].position.x, tile->vertices[0].position.y);
            polygon.AddPoint(tile->vertices[1].position.x, tile->vertices[1].position.y);
            polygon.AddPoint(tile->vertices[3].position.x, tile->vertices[3].position.y);
            polygon.AddPoint(tile->vertices[2].position.x, tile->vertices[2].position.y);
            if (tile->cost_map != 0)
                GeometryViz::FillPolygon(canvas, polygon, false, JetColorMap::Transform(tile->cost_map));
        }
    }

    CurvilinearGridViz::DrawCurvilinearGrid(canvas, grid);
}

template <typename GridType>
void DrawCurvilinearGridGrayscaleCost(CvCanvas &canvas, const GridType &grid, cv::Scalar ln_color = CvColors::lime_color, int32_t thickness = 1)
{
    for (auto &tile_row : grid.grid_tiles_)
    {
        for (auto tile : tile_row)
        {
            Polygon polygon;
            polygon.AddPoint(tile->vertices[0].position.x, tile->vertices[0].position.y);
            polygon.AddPoint(tile->vertices[1].position.x, tile->vertices[1].position.y);
            polygon.AddPoint(tile->vertices[3].position.x, tile->vertices[3].position.y);
            polygon.AddPoint(tile->vertices[2].position.x, tile->vertices[2].position.y);
            if (tile->cost_map != 0)
            {
                double color_val = (1.0 - tile->cost_map) * 255;
                GeometryViz::FillPolygon(canvas, polygon, false, cv::Scalar(color_val, color_val, color_val));
            }
        }
    }

    CurvilinearGridViz::DrawCurvilinearGrid(canvas, grid);
}
}; // namespace CurvilinearGridViz
} // namespace autodrive

#endif /* CURVILINEAR_GRID_DRAW_HPP */
