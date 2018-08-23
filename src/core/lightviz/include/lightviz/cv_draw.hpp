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
    static const cv::Scalar jet_colormap_lowest;

    static const cv::Scalar black_color;
    static const cv::Scalar white_color;
    static const cv::Scalar red_color;
    static const cv::Scalar lime_color;
    static const cv::Scalar blue_color;
    static const cv::Scalar yellow_color;
    static const cv::Scalar cyan_color;
    static const cv::Scalar magenta_color;
    static const cv::Scalar silver_color;
    static const cv::Scalar gray_color;
    static const cv::Scalar maroon_color;
    static const cv::Scalar olive_color;
    static const cv::Scalar green_color;
    static const cv::Scalar purple_color;
    static const cv::Scalar teal_color;
    static const cv::Scalar navy_color;
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

/******************** Transformations *********************/
cv::Scalar JetPaletteTransform(double val);
void JetPaletteTransform(double val, double &r, double &g, double &b);

/******************** Draw Primitives *********************/
void DrawPoint(cv::Mat img, cv::Point pos, const cv::Scalar &color = LVColors::default_pt_color, int thick = 1);
void DrawLine(cv::Mat img, cv::Point pt1, cv::Point pt2, const cv::Scalar &color = LVColors::default_ln_color, int thick = 1);
void DrawArrow(cv::Mat img, cv::Point base_pos, double length, double angle, const cv::Scalar &color = LVColors::default_pt_color, int thick = 1);
void DrawArrow(cv::Mat img, cv::Point base_pos, cv::Point tip_pos, const cv::Scalar &color = LVColors::red_color, int thick = 1);

void WriteText(cv::Mat img, std::string text, cv::Point pos, const cv::Scalar &color = LVColors::black_color, double scale = 1, int thick = 1);
} // namespace LightViz
} // namespace librav

#endif /* CV_DRAW_HPP */
