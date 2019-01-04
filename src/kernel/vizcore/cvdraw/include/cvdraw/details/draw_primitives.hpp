/* 
 * draw_primitives.hpp
 * 
 * Created on: Jan 04, 2019 10:12
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef DRAW_PRIMITIVES_HPP
#define DRAW_PRIMITIVES_HPP

#include "cvdraw/details/cvdraw_headers.hpp"
#include "cvdraw/details/cv_colors.hpp"

namespace librav
{
void DrawPoint(cv::Mat img, cv::Point pos, const cv::Scalar &color = CvDrawColors::default_pt_color, int thick = 1);
void DrawLine(cv::Mat img, cv::Point pt1, cv::Point pt2, const cv::Scalar &color = CvDrawColors::default_ln_color, int thick = 1);
void DrawArrow(cv::Mat img, cv::Point base_pos, double length, double angle, const cv::Scalar &color = CvDrawColors::default_pt_color, int thick = 1);
void DrawArrow(cv::Mat img, cv::Point base_pos, cv::Point tip_pos, const cv::Scalar &color = CvDrawColors::red_color, int thick = 1);

void WriteText(cv::Mat img, std::string text, cv::Point pos, const cv::Scalar &color = CvDrawColors::black_color, double scale = 1, int thick = 1);

} // namespace librav

#endif /* DRAW_PRIMITIVES_HPP */
