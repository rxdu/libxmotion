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

#include "cvdraw/cvdraw.hpp"

#include "decomp/square_grid.hpp"
#include "graph/graph.hpp"
#include "decomp/square_grid_draw.hpp"

namespace ivnav
{
namespace LightViz
{
template <typename GridType>
void ShowSquareGrid(GridType *grid, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false)
{
    CvCanvas canvas = SquareGridViz::CreateCanvas(grid, pixel_per_unit);

    SquareGridViz::DrawGridCell(canvas, grid);
    SquareGridViz::DrawGridNet(canvas, grid);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

template <typename GridType, typename GridCellType>
void ShowSquareGridPath(GridType *grid, const std::vector<GridCellType *> &path, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false)
{
    CvCanvas canvas = SquareGridViz::CreateCanvas(grid, pixel_per_unit);

    SquareGridViz::DrawGridCell(canvas, grid);
    SquareGridViz::DrawGridPathStartGoal(canvas, path);
    SquareGridViz::DrawGridNet(canvas, grid);
    SquareGridViz::DrawGridPath(canvas, path);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

template <typename GridType, typename GridCellType>
void ShowSquareGridGraph(GridType *grid, Graph<GridCellType *> *graph, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false)
{
    CvCanvas canvas = SquareGridViz::CreateCanvas(grid, pixel_per_unit);

    SquareGridViz::DrawGridCell(canvas, grid);
    SquareGridViz::DrawGridNet(canvas, grid);
    SquareGridViz::DrawGridGraph(canvas, grid, graph);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

template <typename GridType>
void ShowSquareGridGraphCost(GridType *grid, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false)
{
    CvCanvas canvas = SquareGridViz::CreateCanvas(grid, pixel_per_unit);

    SquareGridViz::DrawGridCell(canvas, grid);
    SquareGridViz::DrawGridCost(canvas, grid);
    SquareGridViz::DrawGridNet(canvas, grid);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

template <typename GridType>
void ShowSquareGridGraphCostOnly(GridType *grid, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false)
{
    CvCanvas canvas = SquareGridViz::CreateCanvas(grid, pixel_per_unit);

    SquareGridViz::DrawGridCell(canvas, grid);
    SquareGridViz::DrawGridCostOnly(canvas, grid);
    SquareGridViz::DrawGridNet(canvas, grid);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}
} // namespace LightViz
} // namespace ivnav

#endif /* GRID_VIZ_HPP */
