/* 
 * image_plot.h
 * 
 * Created on: Nov 27, 2017 16:47
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef IMAGE_PLOT_H
#define IMAGE_PLOT_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace librav
{
namespace FastPlot
{
    // Read image from a file
    cv::Mat ReadImageFile(std::string img_file);
    cv::Mat ReadColorImage(std::string img_file);
    cv::Mat ReadGreyscaleImage(std::string img_file);

    // Display image
    void ShowImage(cv::Mat img, std::string window_name = "Image");
    void ShowMatrixAsImage(const Eigen::MatrixXd& matrix, std::string window_name = "Matrix Image");
    void ShowMatrixAsColorMap(const Eigen::MatrixXd& matrix, std::string window_name = "Matrix Color Map");
}
}

#endif /* IMAGE_PLOT_H */
