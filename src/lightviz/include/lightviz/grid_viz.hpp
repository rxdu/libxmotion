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

#include "decomp/square_grid.hpp"
#include "lightviz/cv_draw.hpp"

namespace librav
{
namespace LightViz
{
cv::Mat CreateSquareGridCanvas(SquareGrid *grid, int32_t pixel_per_unit = 10);
cv::Mat SquareGridCellViz(cv::Mat canvas, SquareGrid *grid, int32_t pixel_per_unit = 10);
cv::Mat SquareGridNetViz(cv::Mat canvas, SquareGrid *grid, int32_t pixel_per_unit = 10);
cv::Mat SquareGridPathStartGoalViz(cv::Mat canvas, const std::vector<SquareCell *> &path, int32_t pixel_per_unit = 10);
cv::Mat SquareGridPathViz(cv::Mat canvas, const std::vector<SquareCell *> &path, int32_t pixel_per_unit = 10);

void ShowSquareGrid(SquareGrid *grid, int32_t pixel_per_unit = 10, std::string window_name = "Square Grid", bool save_img = false);
void ShowSquareGridPath(SquareGrid *grid, const std::vector<SquareCell *> &path, int32_t pixel_per_unit = 10, std::string window_name = "Square Grid", bool save_img = false);
}
}

#endif /* GRID_VIZ_HPP */
