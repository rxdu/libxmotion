/* 
 * grid_viz.hpp
 * 
 * Created on: Mar 29, 2018 22:47
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef GRID_VIZ_HPP
#define GRID_VIZ_HPP

#include <cstdint>

#include "lightviz/cv_draw.hpp"
#include "lightviz/grid_draw.hpp"

#include "decomp/square_grid.hpp"
#include "graph/graph.hpp"

namespace librav
{
namespace LightViz
{
template <typename GridType>
void ShowSquareGrid(GridType *grid, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false)
{
    SquareGridDraw gdraw(pixel_per_unit);

    cv::Mat canvas = gdraw.CreateGridCanvas(grid);
    canvas = gdraw.DrawGridCell(canvas, grid);
    canvas = gdraw.DrawGridNet(canvas, grid);

    ShowImage(canvas, window_name, save_img);
}

template <typename GridType, typename GridCellType>
void ShowSquareGridPath(GridType *grid, const std::vector<GridCellType *> &path, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false)
{
    SquareGridDraw gdraw(pixel_per_unit);

    cv::Mat canvas = gdraw.CreateGridCanvas(grid);
    canvas = gdraw.DrawGridCell(canvas, grid);
    canvas = gdraw.DrawGridPathStartGoal(canvas, path);
    canvas = gdraw.DrawGridNet(canvas, grid);
    canvas = gdraw.DrawGridPath(canvas, path);

    ShowImage(canvas, window_name, save_img);
}

template <typename GridType, typename GridCellType>
void ShowSquareGridGraph(GridType *grid, Graph_t<GridCellType *> *graph, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false)
{
    SquareGridDraw gdraw(pixel_per_unit);

    cv::Mat canvas = gdraw.CreateGridCanvas(grid);
    canvas = gdraw.DrawGridCell(canvas, grid);
    canvas = gdraw.DrawGridNet(canvas, grid);
    canvas = gdraw.DrawGridGraph(canvas, grid, graph);

    ShowImage(canvas, window_name, save_img);
}

template <typename GridType, typename GridCellType>
void ShowSquareGridGraphCost(GridType *grid, Graph_t<GridCellType *> *graph, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false)
{
    SquareGridDraw gdraw(pixel_per_unit);

    cv::Mat canvas = gdraw.CreateGridCanvas(grid);
    canvas = gdraw.DrawGridCell(canvas, grid);
    canvas = gdraw.DrawGridCost(canvas, grid);
    canvas = gdraw.DrawGridNet(canvas, grid);

    ShowImage(canvas, window_name, save_img);
}
}
}

#endif /* GRID_VIZ_HPP */
