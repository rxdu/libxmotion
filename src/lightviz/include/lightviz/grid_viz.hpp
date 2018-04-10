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

#include "decomp/square_grid.hpp"
#include "graph/graph.hpp"

namespace librav
{
namespace LightViz
{
// convenience functions
void ShowSquareGrid(SquareGrid *grid, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false);
void ShowSquareGridPath(SquareGrid *grid, const std::vector<SquareCell *> &path, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false);

void ShowSquareGridGraph(SquareGrid *grid, Graph_t<SquareCell *> *graph, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false);
void ShowSquareGridGraphCost(SquareGrid *grid, Graph_t<SquareCell *> *graph, int32_t pixel_per_unit = 100, std::string window_name = "Square Grid", bool save_img = false);
}
}

#endif /* GRID_VIZ_HPP */
