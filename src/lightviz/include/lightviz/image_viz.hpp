/* 
 * image_viz.hpp
 * 
 * Created on: Mar 28, 2018 14:44
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef IMAGE_VIZ_HPP
#define IMAGE_VIZ_HPP

#include <string>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace librav
{
namespace LightViz
{
// Display image
void ShowImage(std::string file_name, std::string window_name = "Image");
void ShowImage(cv::Mat img, std::string window_name = "Image", bool save_img = false);

void ShowMatrixAsImage(const Eigen::MatrixXd &matrix, std::string window_name = "Matrix Image", bool save_img = false);
void ShowMatrixAsColorMap(const Eigen::MatrixXd &matrix, std::string window_name = "Matrix Color Map", bool save_img = false);
}
}
#endif /* IMAGE_VIZ_HPP */
