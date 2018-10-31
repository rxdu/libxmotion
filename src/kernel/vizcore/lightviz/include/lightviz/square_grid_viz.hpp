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

#include "canvas/cv_draw.hpp"
#include "lightviz/details/square_grid_draw.hpp"

#include "decomp/square_grid.hpp"
#include "graph/graph.hpp"

namespace librav
{
namespace LightViz
{
template <typename GridType>
void ShowSquareGrid(GridType *grid, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false)
{
    CartesianCanvas canvas(pixel_per_unit);
    SquareGridDraw gdraw(canvas);

    gdraw.SetupCanvas(grid, pixel_per_unit);

    gdraw.DrawGridCell(grid);
    gdraw.DrawGridNet(grid);

    CvDraw::ShowImage(canvas.paint_area, window_name, save_img);
}

template <typename GridType, typename GridCellType>
void ShowSquareGridPath(GridType *grid, const std::vector<GridCellType *> &path, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false)
{
    CartesianCanvas canvas(pixel_per_unit);
    SquareGridDraw gdraw(canvas);

    gdraw.SetupCanvas(grid, pixel_per_unit);

    gdraw.DrawGridCell(grid);
    gdraw.DrawGridPathStartGoal(path);
    gdraw.DrawGridNet(grid);
    gdraw.DrawGridPath(path);

    CvDraw::ShowImage(canvas.paint_area, window_name, save_img);
}

template <typename GridType, typename GridCellType>
void ShowSquareGridGraph(GridType *grid, Graph<GridCellType *> *graph, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false)
{
    CartesianCanvas canvas(pixel_per_unit);
    SquareGridDraw gdraw(canvas);

    gdraw.SetupCanvas(grid, pixel_per_unit);

    gdraw.DrawGridCell(grid);
    gdraw.DrawGridNet(grid);
    gdraw.DrawGridGraph(grid, graph);

    CvDraw::ShowImage(canvas.paint_area, window_name, save_img);
}

template <typename GridType>
void ShowSquareGridGraphCost(GridType *grid, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false)
{
    CartesianCanvas canvas(pixel_per_unit);
    SquareGridDraw gdraw(canvas);

    gdraw.SetupCanvas(grid, pixel_per_unit);

    gdraw.DrawGridCell(grid);
    gdraw.DrawGridCost(grid);
    gdraw.DrawGridNet(grid);

    CvDraw::ShowImage(canvas.paint_area, window_name, save_img);
}

template <typename GridType>
void ShowSquareGridGraphCostOnly(GridType *grid, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false)
{
    CartesianCanvas canvas(pixel_per_unit);
    SquareGridDraw gdraw(canvas);

    gdraw.SetupCanvas(grid, pixel_per_unit);

    gdraw.DrawGridCell(grid);
    gdraw.DrawGridCostOnly(grid);
    gdraw.DrawGridNet(grid);

    CvDraw::ShowImage(canvas.paint_area, window_name, save_img);
}
} // namespace LightViz
} // namespace librav

#endif /* GRID_VIZ_HPP */
