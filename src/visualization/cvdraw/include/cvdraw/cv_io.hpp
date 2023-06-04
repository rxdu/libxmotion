/*
 * cv_io.hpp
 *
 * Created on: Mar 28, 2018 14:44
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CV_IO_HPP
#define CV_IO_HPP

#include <string>

#include <opencv2/opencv.hpp>

#include "cvdraw/cv_colors.hpp"
#include "cvdraw/color_maps.hpp"
#include "cvdraw/cv_canvas.hpp"

namespace xmotion {
namespace CvIO {
/******************* Basic Input/Output *******************/
// Read image from file
cv::Mat ReadImageFile(std::string img_file);
cv::Mat ReadColorImage(std::string img_file);
cv::Mat ReadGrayscaleImage(std::string img_file);

// Display image
void ShowImage(std::string file_name, std::string window_name = "Image");
void ShowImage(cv::Mat img, std::string window_name = "Image",
               bool save_img = false);
void ShowImageFrame(cv::Mat img, std::string window_name = "Image",
                    int32_t frame_period_ms = 50);
}  // namespace CvIO
}  // namespace xmotion

#endif /* CV_IO_HPP */
