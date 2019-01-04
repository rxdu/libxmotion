/* 
 * cv_draw.hpp
 * 
 * Created on: Mar 28, 2018 14:44
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CV_DRAW_HPP
#define CV_DRAW_HPP

#include <string>

#include <opencv2/opencv.hpp>

#include "cvdraw/details/cv_colors.hpp"
#include "cvdraw/details/color_maps.hpp"

namespace librav
{
namespace CvDraw
{
/******************* Basic Input/Output *******************/
// Read image from file
cv::Mat ReadImageFile(std::string img_file);
cv::Mat ReadColorImage(std::string img_file);
cv::Mat ReadGrayscaleImage(std::string img_file);

// Display image
void ShowImage(std::string file_name, std::string window_name = "Image");
void ShowImage(cv::Mat img, std::string window_name = "Image", bool save_img = false);
void ShowImageFrame(cv::Mat img, std::string window_name = "Image", int32_t frame_period_ms = 50);

/******************** Draw Primitives *********************/
void DrawPoint(cv::Mat img, cv::Point pos, const cv::Scalar &color = CvDrawColors::default_pt_color, int thick = 1);
void DrawLine(cv::Mat img, cv::Point pt1, cv::Point pt2, const cv::Scalar &color = CvDrawColors::default_ln_color, int thick = 1);
void DrawArrow(cv::Mat img, cv::Point base_pos, double length, double angle, const cv::Scalar &color = CvDrawColors::default_pt_color, int thick = 1);
void DrawArrow(cv::Mat img, cv::Point base_pos, cv::Point tip_pos, const cv::Scalar &color = CvDrawColors::red_color, int thick = 1);

void WriteText(cv::Mat img, std::string text, cv::Point pos, const cv::Scalar &color = CvDrawColors::black_color, double scale = 1, int thick = 1);
} // namespace CvDraw
} // namespace librav

#endif /* CV_DRAW_HPP */
