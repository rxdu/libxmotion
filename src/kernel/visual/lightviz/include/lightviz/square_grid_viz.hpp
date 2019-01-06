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
#include "coreviz/coreviz.hpp"

#include "decomp/square_grid.hpp"
#include "graph/graph.hpp"

namespace librav
{
namespace LightViz
{
template <typename GridType>
void ShowSquareGrid(GridType *grid, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false)
{
    CvCanvas canvas = SquareGridDraw::CreateCanvas(grid, pixel_per_unit);

    SquareGridDraw::DrawGridCell(canvas, grid);
    SquareGridDraw::DrawGridNet(canvas, grid);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

template <typename GridType, typename GridCellType>
void ShowSquareGridPath(GridType *grid, const std::vector<GridCellType *> &path, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false)
{
    CvCanvas canvas = SquareGridDraw::CreateCanvas(grid, pixel_per_unit);

    SquareGridDraw::DrawGridCell(canvas, grid);
    SquareGridDraw::DrawGridPathStartGoal(canvas, path);
    SquareGridDraw::DrawGridNet(canvas, grid);
    SquareGridDraw::DrawGridPath(canvas, path);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

template <typename GridType, typename GridCellType>
void ShowSquareGridGraph(GridType *grid, Graph<GridCellType *> *graph, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false)
{
    CvCanvas canvas = SquareGridDraw::CreateCanvas(grid, pixel_per_unit);

    SquareGridDraw::DrawGridCell(canvas, grid);
    SquareGridDraw::DrawGridNet(canvas, grid);
    SquareGridDraw::DrawGridGraph(canvas, grid, graph);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

template <typename GridType>
void ShowSquareGridGraphCost(GridType *grid, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false)
{
    CvCanvas canvas = SquareGridDraw::CreateCanvas(grid, pixel_per_unit);

    SquareGridDraw::DrawGridCell(canvas, grid);
    SquareGridDraw::DrawGridCost(canvas, grid);
    SquareGridDraw::DrawGridNet(canvas, grid);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

template <typename GridType>
void ShowSquareGridGraphCostOnly(GridType *grid, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false)
{
    CvCanvas canvas = SquareGridDraw::CreateCanvas(grid, pixel_per_unit);

    SquareGridDraw::DrawGridCell(canvas, grid);
    SquareGridDraw::DrawGridCostOnly(canvas, grid);
    SquareGridDraw::DrawGridNet(canvas, grid);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}
} // namespace LightViz
} // namespace librav

#endif /* GRID_VIZ_HPP */
