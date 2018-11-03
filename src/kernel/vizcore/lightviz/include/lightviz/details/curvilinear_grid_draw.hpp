/* 
 * curvilinear_grid_draw.hpp
 * 
 * Created on: Nov 01, 2018 10:19
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef CURVILINEAR_GRID_DRAW_HPP
#define CURVILINEAR_GRID_DRAW_HPP

#include <cstdint>
#include <functional>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "lightviz/details/cartesian_canvas.hpp"
#include "lightviz/details/geometry_draw.hpp"

#include "decomp/curvilinear_grid.hpp"
#include "canvas/cv_draw.hpp"

namespace librav
{
class CurvilinearGridDraw
{
  public:
    CurvilinearGridDraw(CartesianCanvas &canvas) : canvas_(canvas), gdraw_(canvas){};

    // geometric grid
    void DrawCurvilinearGrid(const CurvilinearGrid &grid, double step = 0.1, bool show_center = false, cv::Scalar ln_color = CvDrawColors::lime_color, int32_t ln_width = 1);
    void DrawCurvilinearGridCost(const CurvilinearGrid &grid, double step = 0.1, bool show_center = false, cv::Scalar ln_color = CvDrawColors::lime_color, int32_t ln_width = 1);
    void DrawCurvilinearGridCostOnly(const CurvilinearGrid &grid, double step = 0.1, bool show_center = false, cv::Scalar ln_color = CvDrawColors::lime_color, int32_t ln_width = 1);
    void DrawCurvilinearGridGrayscaleCost(const CurvilinearGrid &grid, double step = 0.1, bool show_center = false, cv::Scalar ln_color = CvDrawColors::lime_color, int32_t ln_width = 1);

  private:
    // internal parameters
    CartesianCanvas &canvas_;
    GeometryDraw gdraw_;
};
} // namespace librav

#endif /* CURVILINEAR_GRID_DRAW_HPP */
