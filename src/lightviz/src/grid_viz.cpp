/* 
 * grid_viz.cpp
 * 
 * Created on: Mar 29, 2018 22:50
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "lightviz/grid_viz.hpp"
#include "lightviz/cv_draw.hpp"

using namespace librav;
using namespace cv;

cv::Mat LightViz::CreateSquareGridCanvas(SquareGrid *grid, int32_t pixel_per_unit)
{
    // create canvas
    int32_t vis_side_size = grid->cell_size_ * pixel_per_unit;
    cv::Mat canvas(grid->SizeY() * vis_side_size, grid->SizeX() * vis_side_size, CV_8UC3, LVColors::bg_color);

    return canvas;
}

cv::Mat LightViz::SquareGridCellViz(cv::Mat canvas, SquareGrid *grid, int32_t pixel_per_unit)
{
    // draw cells
    for (int32_t y = 0; y < grid->SizeY(); ++y)
        for (int32_t x = 0; x < grid->SizeX(); ++x)
        {
            auto cell = grid->GetCell(x, y);
            if (cell->label == SquareCellLabel::OCCUPIED)
            {
                Range rngx(cell->vertices[0].x * pixel_per_unit, cell->vertices[1].x * pixel_per_unit);
                Range rngy(cell->vertices[0].y * pixel_per_unit, cell->vertices[2].y * pixel_per_unit);
                canvas(rngy, rngx) = LVColors::obs_color;
            }
        }

    return canvas;
}

cv::Mat LightViz::SquareGridNetViz(cv::Mat canvas, SquareGrid *grid, int32_t pixel_per_unit)
{
    // draw horizontal lines
    for (int32_t y = 1; y < grid->SizeY(); ++y)
    {
        auto first_vertex = grid->GetCell(0, y)->vertices[0];
        auto last_vertex = grid->GetCell(grid->SizeX() - 1, y)->vertices[1];

        cv::Point pt1(first_vertex.x * pixel_per_unit, first_vertex.y * pixel_per_unit);
        cv::Point pt2(last_vertex.x * pixel_per_unit, last_vertex.y * pixel_per_unit);

        DrawLine(canvas, pt1, pt2);
    }

    // draw vertical lines
    for (int32_t x = 1; x < grid->SizeX(); ++x)
    {
        auto first_vertex = grid->GetCell(x, 0)->vertices[0];
        auto last_vertex = grid->GetCell(x, grid->SizeY() - 1)->vertices[2];

        cv::Point pt1(first_vertex.x * pixel_per_unit, first_vertex.y * pixel_per_unit);
        cv::Point pt2(last_vertex.x * pixel_per_unit, last_vertex.y * pixel_per_unit);

        DrawLine(canvas, pt1, pt2);
    }

    return canvas;
}

void LightViz::ShowSquareGrid(SquareGrid *grid, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    cv::Mat canvas = CreateSquareGridCanvas(grid, pixel_per_unit);
    canvas = SquareGridCellViz(canvas, grid, pixel_per_unit);
    canvas = SquareGridNetViz(canvas, grid, pixel_per_unit);

    ShowImage(canvas, window_name, save_img);
}

cv::Mat LightViz::SquareGridPathStartGoalViz(cv::Mat canvas, const std::vector<SquareCell *> &path, int32_t pixel_per_unit)
{
    Range srngx(path.front()->vertices[0].x * pixel_per_unit, path.front()->vertices[1].x * pixel_per_unit);
    Range srngy(path.front()->vertices[0].y * pixel_per_unit, path.front()->vertices[2].y * pixel_per_unit);
    canvas(srngy, srngx) = LVColors::start_color;

    Range grngx(path.back()->vertices[0].x * pixel_per_unit, path.back()->vertices[1].x * pixel_per_unit);
    Range grngy(path.back()->vertices[0].y * pixel_per_unit, path.back()->vertices[2].y * pixel_per_unit);
    canvas(grngy, grngx) = LVColors::finish_color;

    return canvas;
}

cv::Mat LightViz::SquareGridPathViz(cv::Mat canvas, const std::vector<SquareCell *> &path, int32_t pixel_per_unit)
{
    for (int i = 0; i < path.size() - 1; ++i)
    {
        auto first_cell = path[i]->center;
        auto next_cell = path[i + 1]->center;

        cv::Point pt1(first_cell.x * pixel_per_unit, first_cell.y * pixel_per_unit);
        cv::Point pt2(next_cell.x * pixel_per_unit, next_cell.y * pixel_per_unit);

        DrawLine(canvas, pt1, pt2, LVColors::intermediate_color);
    }

    return canvas;
}

void LightViz::ShowSquareGridPath(SquareGrid *grid, const std::vector<SquareCell *> &path, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    cv::Mat canvas = CreateSquareGridCanvas(grid, pixel_per_unit);
    canvas = SquareGridCellViz(canvas, grid, pixel_per_unit);
    canvas = SquareGridPathStartGoalViz(canvas, path, pixel_per_unit);
    canvas = SquareGridNetViz(canvas, grid, pixel_per_unit);
    canvas = SquareGridPathViz(canvas, path, pixel_per_unit);

    ShowImage(canvas, window_name, save_img);
}
