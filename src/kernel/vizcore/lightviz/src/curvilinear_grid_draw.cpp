/* 
 * curvilinear_grid_draw.cpp
 * 
 * Created on: Nov 01, 2018 10:20
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "lightviz/details/curvilinear_grid_draw.hpp"

#include <cassert>

#include "lightviz/details/matrix_draw.hpp"
#include "geometry/polygon.hpp"

using namespace librav;
using namespace LightViz;
using namespace CvDraw;

void CurvilinearGridDraw::DrawCurvilinearGrid(const CurvilinearGrid &grid, double step, bool show_center, cv::Scalar ln_color, int32_t ln_width)
{
    // draw center line
    if (show_center)
        gdraw_.DrawParametricCurve(grid.curve_, step, CvDrawColors::gray_color, ln_width);

    // draw normal lines
    auto spt1 = canvas_.ConvertCartisianToPixel(grid.grid_tiles_.front().back()->vertices[2].position.x, grid.grid_tiles_.front().back()->vertices[2].position.y);
    auto spt2 = canvas_.ConvertCartisianToPixel(grid.grid_tiles_.front().front()->vertices[3].position.x, grid.grid_tiles_.front().front()->vertices[3].position.y);
    DrawLine(canvas_.paint_area, cv::Point(spt1.x, spt1.y), cv::Point(spt2.x, spt2.y), ln_color, ln_width);
    for (auto &row : grid.grid_tiles_)
    {
        auto pt1 = canvas_.ConvertCartisianToPixel(row.back()->vertices[0].position.x, row.back()->vertices[0].position.y);
        auto pt2 = canvas_.ConvertCartisianToPixel(row.front()->vertices[1].position.x, row.front()->vertices[1].position.y);
        DrawLine(canvas_.paint_area, cv::Point(pt1.x, pt1.y), cv::Point(pt2.x, pt2.y), ln_color, ln_width);
    }

    // draw tangential lines
    for (auto &row : grid.grid_tiles_)
    {
        for (auto &cell : row)
        {
            auto pt1 = canvas_.ConvertCartisianToPixel(cell->vertices[1].position.x, cell->vertices[1].position.y);
            auto pt2 = canvas_.ConvertCartisianToPixel(cell->vertices[3].position.x, cell->vertices[3].position.y);
            DrawLine(canvas_.paint_area, cv::Point(pt1.x, pt1.y), cv::Point(pt2.x, pt2.y), ln_color, ln_width);
        }
        auto fpt1 = canvas_.ConvertCartisianToPixel(row.back()->vertices[0].position.x, row.back()->vertices[0].position.y);
        auto fpt2 = canvas_.ConvertCartisianToPixel(row.back()->vertices[2].position.x, row.back()->vertices[2].position.y);
        DrawLine(canvas_.paint_area, cv::Point(fpt1.x, fpt1.y), cv::Point(fpt2.x, fpt2.y), ln_color, ln_width);
    }
}

void CurvilinearGridDraw::DrawCurvilinearGridCost(const CurvilinearGrid &grid, double step, bool show_center, cv::Scalar ln_color, int32_t ln_width)
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
            gdraw_.DrawFilledPolygon(polygon, false, CvDraw::JetPaletteTransform(tile->cost_map));
        }
    }

    DrawCurvilinearGrid(grid, step, show_center);
}

void CurvilinearGridDraw::DrawCurvilinearGridCostOnly(const CurvilinearGrid &grid, double step, bool show_center, cv::Scalar ln_color, int32_t ln_width)
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
                gdraw_.DrawFilledPolygon(polygon, false, CvDraw::JetPaletteTransform(tile->cost_map));
        }
    }

    DrawCurvilinearGrid(grid, step, show_center);
}

void CurvilinearGridDraw::DrawCurvilinearGridGrayscaleCost(const CurvilinearGrid &grid, double step, bool show_center, cv::Scalar ln_color, int32_t ln_width)
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
                gdraw_.DrawFilledPolygon(polygon, false, cv::Scalar(color_val, color_val, color_val));
            }
        }
    }

    DrawCurvilinearGrid(grid, step, show_center);
}