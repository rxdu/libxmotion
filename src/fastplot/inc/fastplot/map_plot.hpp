/* 
 * map_plot.h
 * 
 * Created on: Dec 29, 2017 14:37
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef MAP_PLOT_H
#define MAP_PLOT_H

#include <memory>
#include <opencv2/opencv.hpp>

#include "geometry/square_grid.hpp"

namespace librav
{
namespace FastPlot
{
    void PlotSquareGrid(std::shared_ptr<SquareGrid> grid, bool show_id = false);
    void CreateSquareGridLayer(std::shared_ptr<SquareGrid> grid, cv::OutputArray _dst, bool show_id = false);
    void AddSquareGridLayer(std::shared_ptr<SquareGrid> grid, cv::InputArray _src, cv::OutputArray _dst);
}
}

#endif /* MAP_PLOT_H */
