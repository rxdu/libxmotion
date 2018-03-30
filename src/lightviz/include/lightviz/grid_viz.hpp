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
cv::Mat CreateSquareGridCanvas(SquareGrid *grid);
cv::Mat DrawSquareGridCell(cv::Mat canvas, SquareGrid *grid);
cv::Mat DrawSquareGridNet(cv::Mat canvas, SquareGrid *grid);
cv::Mat DrawSquareGridPathStartGoal(cv::Mat canvas, const std::vector<SquareCell *> &path);
cv::Mat DrawSquareGridPath(cv::Mat canvas, const std::vector<SquareCell *> &path);

void ShowSquareGrid(SquareGrid *grid, int32_t pixel_per_unit = 10, std::string window_name = "Square Grid", bool save_img = false);
void ShowSquareGridPath(SquareGrid *grid, const std::vector<SquareCell *> &path, int32_t pixel_per_unit = 10, std::string window_name = "Square Grid", bool save_img = false);
}
}

#endif /* GRID_VIZ_HPP */
