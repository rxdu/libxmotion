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
#include "lightviz/grid_draw.hpp"

using namespace librav;
using namespace cv;

void LightViz::ShowSquareGrid(SquareGrid *grid, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GridDraw gdraw(pixel_per_unit);

    cv::Mat canvas = gdraw.CreateSquareGridCanvas(grid);
    canvas = gdraw.DrawSquareGridCell(canvas, grid);
    canvas = gdraw.DrawSquareGridNet(canvas, grid);

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowSquareGridPath(SquareGrid *grid, const std::vector<SquareCell *> &path, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GridDraw gdraw(pixel_per_unit);

    cv::Mat canvas = gdraw.CreateSquareGridCanvas(grid);
    canvas = gdraw.DrawSquareGridCell(canvas, grid);
    canvas = gdraw.DrawSquareGridPathStartGoal(canvas, path);
    canvas = gdraw.DrawSquareGridNet(canvas, grid);
    canvas = gdraw.DrawSquareGridPath(canvas, path);

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowSquareGridGraph(SquareGrid *grid, Graph_t<SquareCell *> *graph, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GridDraw gdraw(pixel_per_unit);

    cv::Mat canvas = gdraw.CreateSquareGridCanvas(grid);
    canvas = gdraw.DrawSquareGridCell(canvas, grid);
    canvas = gdraw.DrawSquareGridNet(canvas, grid);
    canvas = gdraw.DrawSquareGridGraph(canvas, grid, graph);

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowSquareGridGraphCost(SquareGrid *grid, Graph_t<SquareCell *> *graph, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GridDraw gdraw(pixel_per_unit);

    cv::Mat canvas = gdraw.CreateSquareGridCanvas(grid);
    canvas = gdraw.DrawSquareGridCell(canvas, grid);
    canvas = gdraw.DrawSquareGridCost(canvas, grid);
    canvas = gdraw.DrawSquareGridNet(canvas, grid);

    ShowImage(canvas, window_name, save_img);
}