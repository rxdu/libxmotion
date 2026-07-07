/*
 * @file style.hpp
 * @brief Shared palette and value->color mappings for the visualization
 * module, so drawers stop inventing colors independently.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_VIZ_STYLE_HPP
#define XMNAV_VIZ_STYLE_HPP

#include <algorithm>

#include "cvdraw/cvdraw.hpp"

namespace xmotion {
namespace viz_style {

// primary roles
inline const cv::Scalar kChosenTrajectory = quickviz::CvColors::blue_color;
inline const cv::Scalar kExecutedTrail = quickviz::CvColors::black_color;
inline const cv::Scalar kGoal = quickviz::CvColors::green_color;
inline const cv::Scalar kObstacle = quickviz::CvColors::gray_color;

// normalized weight [0,1] -> candidate color (dim gray-red -> strong red)
inline cv::Scalar WeightColor(double w) {
  const double a = std::min(1.0, std::max(0.0, w));
  const int shade = 230 - static_cast<int>(190.0 * a);
  return cv::Scalar(shade, shade, 255);
}

}  // namespace viz_style
}  // namespace xmotion

#endif  // XMNAV_VIZ_STYLE_HPP
