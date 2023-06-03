/*
 * color_maps.hpp
 *
 * Created on: Jan 04, 2019 09:51
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef COLOR_MAPS_HPP
#define COLOR_MAPS_HPP

#include <opencv2/opencv.hpp>

namespace xmotion {
namespace JetColorMap {
// Input range: 0-1, Output range: 0-255 (OpenCV color)
cv::Scalar Transform(double val);
void Transform(double val, double &r, double &g, double &b);
};  // namespace JetColorMap
}  // namespace xmotion

#endif /* COLOR_MAPS_HPP */
