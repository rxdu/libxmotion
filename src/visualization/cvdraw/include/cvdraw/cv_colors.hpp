/*
 * cv_colors.hpp
 *
 * Created on: Jan 04, 2019 09:41
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef CV_COLORS_HPP
#define CV_COLORS_HPP

#include <opencv2/opencv.hpp>

namespace xmotion {
/********************* Common Colors *********************/
struct CvColors {
  static const cv::Scalar default_pt_color;    // default point color
  static const cv::Scalar default_ln_color;    // default line color
  static const cv::Scalar default_area_color;  // default area color

  static const cv::Scalar bg_color;            // background color
  static const cv::Scalar ln_color;            // line color
  static const cv::Scalar obs_color;           // obstacle color
  static const cv::Scalar aoi_color;           // area of interest color
  static const cv::Scalar start_color;         // starting cell color
  static const cv::Scalar intermediate_color;  // intermediate cell color
  static const cv::Scalar finish_color;        // finishing cell color
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

  static const cv::Scalar orange_color;
  static const cv::Scalar honeydew_color;
  static const cv::Scalar palegreen_color;
};
}  // namespace xmotion

#endif /* CV_COLORS_HPP */
