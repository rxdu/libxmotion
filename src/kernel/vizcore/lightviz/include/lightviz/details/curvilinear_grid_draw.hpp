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

#include "lightviz/details/cartesian_canvas.hpp"
#include "lightviz/details/geometry_draw.hpp"

#include "decomp/curvilinear_grid.hpp"
#include "canvas/cv_draw.hpp"

namespace librav
{
class CurvilinearGridDraw
{
  public:
    CurvilinearGridDraw(CartesianCanvas &canvas) : canvas_(canvas), gdraw_(canvas){};

    // geometric grid
    template <typename GridType>
    void DrawCurvilinearGrid(const GridType &grid, cv::Scalar ln_color = CvDrawColors::lime_color, int32_t ln_width = 1)
    {
        // draw normal lines
        auto spt1 = canvas_.ConvertCartisianToPixel(grid.grid_tiles_.front().back()->vertices[2].position.x, grid.grid_tiles_.front().back()->vertices[2].position.y);
        auto spt2 = canvas_.ConvertCartisianToPixel(grid.grid_tiles_.front().front()->vertices[3].position.x, grid.grid_tiles_.front().front()->vertices[3].position.y);
        CvDraw::DrawLine(canvas_.paint_area, cv::Point(spt1.x, spt1.y), cv::Point(spt2.x, spt2.y), ln_color, ln_width);
        for (auto &row : grid.grid_tiles_)
        {
            auto pt1 = canvas_.ConvertCartisianToPixel(row.back()->vertices[0].position.x, row.back()->vertices[0].position.y);
            auto pt2 = canvas_.ConvertCartisianToPixel(row.front()->vertices[1].position.x, row.front()->vertices[1].position.y);
            CvDraw::DrawLine(canvas_.paint_area, cv::Point(pt1.x, pt1.y), cv::Point(pt2.x, pt2.y), ln_color, ln_width);
        }

        // draw tangential lines
        for (auto &row : grid.grid_tiles_)
        {
            for (auto &cell : row)
            {
                auto pt1 = canvas_.ConvertCartisianToPixel(cell->vertices[1].position.x, cell->vertices[1].position.y);
                auto pt2 = canvas_.ConvertCartisianToPixel(cell->vertices[3].position.x, cell->vertices[3].position.y);
                CvDraw::DrawLine(canvas_.paint_area, cv::Point(pt1.x, pt1.y), cv::Point(pt2.x, pt2.y), ln_color, ln_width);
            }
            auto fpt1 = canvas_.ConvertCartisianToPixel(row.back()->vertices[0].position.x, row.back()->vertices[0].position.y);
            auto fpt2 = canvas_.ConvertCartisianToPixel(row.back()->vertices[2].position.x, row.back()->vertices[2].position.y);
            CvDraw::DrawLine(canvas_.paint_area, cv::Point(fpt1.x, fpt1.y), cv::Point(fpt2.x, fpt2.y), ln_color, ln_width);
        }
    }

    template <typename GridType>
    void DrawFilledCurvilinearGrid(const GridType &grid, cv::Scalar fill_color = CvDrawColors::lime_color, int32_t ln_width = 1)
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

        gdraw_.DrawFilledPolygon(polygon, false, fill_color);
    }

    template <typename GridType>
    void DrawCurvilinearGridCost(const GridType &grid, cv::Scalar ln_color = CvDrawColors::lime_color, int32_t ln_width = 1)
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

        DrawCurvilinearGrid(grid);
    }

    template <typename GridType>
    void DrawCurvilinearGridCostOnly(const GridType &grid, cv::Scalar ln_color = CvDrawColors::lime_color, int32_t ln_width = 1)
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

        DrawCurvilinearGrid(grid);
    }

    template <typename GridType>
    void DrawCurvilinearGridGrayscaleCost(const GridType &grid, cv::Scalar ln_color = CvDrawColors::lime_color, int32_t ln_width = 1)
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

        DrawCurvilinearGrid(grid);
    }

    cv::Mat GetPaintArea() { return canvas_.paint_area; }

  private:
    // internal parameters
    CartesianCanvas &canvas_;
    GeometryDraw gdraw_;
}; // namespace librav
} // namespace librav

#endif /* CURVILINEAR_GRID_DRAW_HPP */
