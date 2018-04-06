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

namespace
{
static int32_t PIXEL_PER_UNIT = 10;
}

cv::Mat LightViz::CreateSquareGridCanvas(SquareGrid *grid)
{
    // create canvas
    int32_t vis_side_size = grid->GetCellSize() * PIXEL_PER_UNIT;
    cv::Mat canvas(grid->SizeY() * vis_side_size, grid->SizeX() * vis_side_size, CV_8UC3, LVColors::bg_color);

    return canvas;
}

cv::Mat LightViz::DrawSquareGridCell(cv::Mat canvas, SquareGrid *grid)
{
    // draw cells
    for (int32_t y = 0; y < grid->SizeY(); ++y)
        for (int32_t x = 0; x < grid->SizeX(); ++x)
        {
            auto cell = grid->GetCell(x, y);
            if (cell->label == SquareCellLabel::OCCUPIED)
            {
                Range rngx(cell->vertices[0].x * PIXEL_PER_UNIT, cell->vertices[1].x * PIXEL_PER_UNIT);
                Range rngy(cell->vertices[0].y * PIXEL_PER_UNIT, cell->vertices[2].y * PIXEL_PER_UNIT);
                canvas(rngy, rngx) = LVColors::obs_color;
            }
        }

    return canvas;
}

cv::Mat LightViz::DrawSquareGridCost(cv::Mat canvas, SquareGrid *grid)
{
    // draw cells
    for (int32_t y = 0; y < grid->SizeY(); ++y)
        for (int32_t x = 0; x < grid->SizeX(); ++x)
        {
            auto cell = grid->GetCell(x, y);
            if (cell->label != SquareCellLabel::OCCUPIED)
            {
                Range rngx(cell->vertices[0].x * PIXEL_PER_UNIT, cell->vertices[1].x * PIXEL_PER_UNIT);
                Range rngy(cell->vertices[0].y * PIXEL_PER_UNIT, cell->vertices[2].y * PIXEL_PER_UNIT);
                canvas(rngy, rngx) = JetPaletteTransform(cell->cost_map);
            }
        }

    return canvas;
}

cv::Mat LightViz::DrawSquareGridNet(cv::Mat canvas, SquareGrid *grid)
{
    // draw horizontal lines
    for (int32_t y = 1; y < grid->SizeY(); ++y)
    {
        auto first_vertex = grid->GetCell(0, y)->vertices[0];
        auto last_vertex = grid->GetCell(grid->SizeX() - 1, y)->vertices[1];

        cv::Point pt1(first_vertex.x * PIXEL_PER_UNIT, first_vertex.y * PIXEL_PER_UNIT);
        cv::Point pt2(last_vertex.x * PIXEL_PER_UNIT, last_vertex.y * PIXEL_PER_UNIT);

        DrawLine(canvas, pt1, pt2);
    }

    // draw vertical lines
    for (int32_t x = 1; x < grid->SizeX(); ++x)
    {
        auto first_vertex = grid->GetCell(x, 0)->vertices[0];
        auto last_vertex = grid->GetCell(x, grid->SizeY() - 1)->vertices[2];

        cv::Point pt1(first_vertex.x * PIXEL_PER_UNIT, first_vertex.y * PIXEL_PER_UNIT);
        cv::Point pt2(last_vertex.x * PIXEL_PER_UNIT, last_vertex.y * PIXEL_PER_UNIT);

        DrawLine(canvas, pt1, pt2);
    }

    return canvas;
}

cv::Mat LightViz::DrawSquareGridPathStartGoal(cv::Mat canvas, const std::vector<SquareCell *> &path)
{
    Range srngx(path.front()->vertices[0].x * PIXEL_PER_UNIT, path.front()->vertices[1].x * PIXEL_PER_UNIT);
    Range srngy(path.front()->vertices[0].y * PIXEL_PER_UNIT, path.front()->vertices[2].y * PIXEL_PER_UNIT);
    canvas(srngy, srngx) = LVColors::start_color;

    Range grngx(path.back()->vertices[0].x * PIXEL_PER_UNIT, path.back()->vertices[1].x * PIXEL_PER_UNIT);
    Range grngy(path.back()->vertices[0].y * PIXEL_PER_UNIT, path.back()->vertices[2].y * PIXEL_PER_UNIT);
    canvas(grngy, grngx) = LVColors::finish_color;

    return canvas;
}

cv::Mat LightViz::DrawSquareGridPath(cv::Mat canvas, const std::vector<SquareCell *> &path)
{
    for (int i = 0; i < path.size() - 1; ++i)
    {
        auto first_cell = path[i]->center;
        auto next_cell = path[i + 1]->center;

        cv::Point pt1(first_cell.x * PIXEL_PER_UNIT, first_cell.y * PIXEL_PER_UNIT);
        cv::Point pt2(next_cell.x * PIXEL_PER_UNIT, next_cell.y * PIXEL_PER_UNIT);

        DrawLine(canvas, pt1, pt2, LVColors::intermediate_color);
    }

    return canvas;
}

cv::Mat LightViz::DrawSquareGridGraph(cv::Mat canvas, SquareGrid *grid, Graph_t<SquareCell *> *graph)
{
    // draw all edges
    auto edges = graph->GetAllEdges();
    for (auto &edge_it : edges)
    {
        auto edge = *edge_it;
        int64_t loc_x1 = edge.src_->state_->center.x * PIXEL_PER_UNIT;
        int64_t loc_y1 = edge.src_->state_->center.y * PIXEL_PER_UNIT;
        int64_t loc_x2 = edge.dst_->state_->center.x * PIXEL_PER_UNIT;
        int64_t loc_y2 = edge.dst_->state_->center.y * PIXEL_PER_UNIT;

        DrawLine(canvas, cv::Point(loc_x1, loc_y1), cv::Point(loc_x2, loc_y2), cv::Scalar(237, 149, 100));
    }

    // draw all vertices
    for (auto vertex = graph->vertex_begin(); vertex != graph->vertex_end(); ++vertex)
    {
        // current vertex center coordinate
        int32_t loc_x = vertex->state_->center.x * PIXEL_PER_UNIT;
        int32_t loc_y = vertex->state_->center.y * PIXEL_PER_UNIT;

        cv::Point center(loc_x, loc_y);
        DrawPoint(canvas, center);

        // if (show_id && vertex->state_->GetUniqueID() % 5 == 0)
        // {
        //     std::string id = std::to_string(vertex->state_->GetUniqueID());
        //     cv::putText(dst, id, cv::Point(loc_x, loc_y), CV_FONT_NORMAL, 0.5, cv::Scalar(204, 204, 102), 1, 1);
        // }
    }

    return canvas;
}

////////////////////////////////////////////////////////////////////////////////////

void LightViz::ShowSquareGrid(SquareGrid *grid, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    PIXEL_PER_UNIT = pixel_per_unit;

    cv::Mat canvas = CreateSquareGridCanvas(grid);
    canvas = DrawSquareGridCell(canvas, grid);
    canvas = DrawSquareGridNet(canvas, grid);

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowSquareGridPath(SquareGrid *grid, const std::vector<SquareCell *> &path, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    PIXEL_PER_UNIT = pixel_per_unit;
    cv::Mat canvas = CreateSquareGridCanvas(grid);
    canvas = DrawSquareGridCell(canvas, grid);
    canvas = DrawSquareGridPathStartGoal(canvas, path);
    canvas = DrawSquareGridNet(canvas, grid);
    canvas = DrawSquareGridPath(canvas, path);

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowSquareGridGraph(SquareGrid *grid, Graph_t<SquareCell *> *graph, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    PIXEL_PER_UNIT = pixel_per_unit;

    cv::Mat canvas = CreateSquareGridCanvas(grid);
    canvas = DrawSquareGridCell(canvas, grid);
    canvas = DrawSquareGridNet(canvas, grid);
    canvas = DrawSquareGridGraph(canvas, grid, graph);

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowSquareGridGraphCost(SquareGrid *grid, Graph_t<SquareCell *> *graph, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    PIXEL_PER_UNIT = pixel_per_unit;

    cv::Mat canvas = CreateSquareGridCanvas(grid);
    canvas = DrawSquareGridCell(canvas, grid);
    canvas = DrawSquareGridCost(canvas, grid);
    canvas = DrawSquareGridNet(canvas, grid);

    ShowImage(canvas, window_name, save_img);
}