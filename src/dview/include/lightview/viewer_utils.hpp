/* 
 * viewer_utils.hpp
 * 
 * Created on: Nov 20, 2018 22:54
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef VIEWER_UTILS_HPP
#define VIEWER_UTILS_HPP

#include "opencv2/opencv.hpp"

namespace ivnav
{
namespace ViewerUtils
{
void ConvertMatToGL(const cv::Mat &src, unsigned int *texID);
} // namespace ViewerUtils
} // namespace ivnav

#endif /* VIEWER_UTILS_HPP */
