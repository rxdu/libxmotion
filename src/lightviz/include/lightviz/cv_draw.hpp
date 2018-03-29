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

namespace librav
{
/********************* Common Colors *********************/
struct LVColors
{
    static const cv::Scalar default_pt_color;   // default point color
    static const cv::Scalar default_ln_color;   // default line color
    static const cv::Scalar default_area_color; // default area color

    static const cv::Scalar bg_color;           // background color
    static const cv::Scalar ln_color;           // line color
    static const cv::Scalar obs_color;          // obstacle color
    static const cv::Scalar aoi_color;          // area of interest color
    static const cv::Scalar start_color;        // starting cell color
    static const cv::Scalar intermediate_color; // intermediate cell color
    static const cv::Scalar finish_color;       // finishing cell color
};

namespace LightViz
{
/******************* Basic Input/Output *******************/
// Read image from file
cv::Mat ReadImageFile(std::string img_file);
cv::Mat ReadColorImage(std::string img_file);
cv::Mat ReadGrayscaleImage(std::string img_file);

// Display image
void ShowImage(std::string file_name, std::string window_name = "Image");
void ShowImage(cv::Mat img, std::string window_name = "Image", bool save_img = false);

/******************** Draw Primitives *********************/
void DrawPoint(cv::Mat img, cv::Point pos, const cv::Scalar &color = LVColors::default_pt_color);
void DrawLine(cv::Mat img, cv::Point pt1, cv::Point pt2, const cv::Scalar &color = LVColors::default_ln_color);
void DrawArrow(cv::Mat img, cv::Point base_pos, double length, double angle, const cv::Scalar &color = LVColors::default_pt_color);
}
}

#endif /* CV_DRAW_HPP */
