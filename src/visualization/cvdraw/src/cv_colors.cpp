/*
 * cv_colors.cpp
 *
 * Created on: Jan 04, 2019 10:15
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "cvdraw/cv_colors.hpp"

using namespace xmotion;
using namespace cv;

/********************* Common Colors *********************/
const cv::Scalar CvColors::default_pt_color = Scalar(0, 0, 255);
const cv::Scalar CvColors::default_ln_color = Scalar(Scalar(0, 0, 0));
const cv::Scalar CvColors::default_area_color = Scalar(0, 255, 255);

const cv::Scalar CvColors::bg_color = Scalar(255, 255, 255);
const cv::Scalar CvColors::ln_color = Scalar(Scalar(0, 0, 0));
const cv::Scalar CvColors::obs_color = Scalar(Scalar(0, 102, 204));
const cv::Scalar CvColors::aoi_color = Scalar(Scalar(0, 255, 255));
const cv::Scalar CvColors::start_color = Scalar(0, 0, 255);
const cv::Scalar CvColors::intermediate_color =
    cv::Scalar(255, 153, 51);  // Scalar(0, 0, 255);
const cv::Scalar CvColors::finish_color =
    Scalar(51, 153, 51);  // Scalar(153, 76, 0);
const cv::Scalar CvColors::jet_colormap_lowest = Scalar(128, 0, 0);

// Reference: https://www.rapidtables.com/web/color/RGB_Color.html
const cv::Scalar CvColors::black_color = Scalar(0, 0, 0);
const cv::Scalar CvColors::white_color = Scalar(255, 255, 255);
const cv::Scalar CvColors::red_color = Scalar(0, 0, 255);
const cv::Scalar CvColors::lime_color = Scalar(0, 255, 0);
const cv::Scalar CvColors::blue_color = Scalar(255, 0, 0);
const cv::Scalar CvColors::yellow_color = Scalar(0, 255, 255);
const cv::Scalar CvColors::cyan_color = Scalar(255, 255, 0);
const cv::Scalar CvColors::magenta_color = Scalar(255, 0, 255);
const cv::Scalar CvColors::silver_color = Scalar(192, 192, 192);
const cv::Scalar CvColors::gray_color = Scalar(128, 128, 128);
const cv::Scalar CvColors::maroon_color = Scalar(0, 0, 128);
const cv::Scalar CvColors::olive_color = Scalar(128, 128, 0);
const cv::Scalar CvColors::green_color = Scalar(0, 128, 0);
const cv::Scalar CvColors::purple_color = Scalar(128, 0, 128);
const cv::Scalar CvColors::teal_color = Scalar(128, 128, 0);
const cv::Scalar CvColors::navy_color = Scalar(128, 0, 0);

const cv::Scalar CvColors::orange_color = Scalar(0, 165, 255);
const cv::Scalar CvColors::honeydew_color = Scalar(240, 255, 240);
const cv::Scalar CvColors::palegreen_color = Scalar(152, 251, 152);
