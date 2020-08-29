/* 
 * lightwidgets.hpp
 * 
 * Created on: Dec 07, 2018 08:04
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LIGHTWIDGETS_HPP
#define LIGHTWIDGETS_HPP

#include "imgui.h"
#include "opencv2/opencv.hpp"

namespace librav
{
namespace LightWidget
{
void DrawOpenCVImageToBackground(cv::Mat img);
}
} // namespace librav

#endif /* LIGHTWIDGETS_HPP */