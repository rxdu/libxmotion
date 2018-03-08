/* 
 * image_plot.cpp
 * 
 * Created on: Nov 27, 2017 16:47
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "fastplot/image_plot.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace librav;
using namespace cv;

cv::Mat FastPlot::ReadImageFile(std::string img_file)
{
    return imread(img_file);
}

cv::Mat FastPlot::ReadColorImage(std::string img_file)
{
    return imread(img_file, CV_LOAD_IMAGE_COLOR);
}

cv::Mat FastPlot::ReadGreyscaleImage(std::string img_file)
{
    return imread(img_file, CV_LOAD_IMAGE_GRAYSCALE);
}

void FastPlot::ShowImage(cv::Mat img, std::string window_name)
{
    namedWindow(window_name, WINDOW_NORMAL); // Create a window for display.
    imshow(window_name, img);                // Show our image inside it.

    waitKey(0); // Wait for a keystroke in the window

    destroyWindow(window_name);
}

void FastPlot::ShowMatrixAsImage(const Eigen::MatrixXd &matrix, std::string window_name)
{
    cv::Mat img;
    eigen2cv(matrix, img);
    namedWindow(window_name, WINDOW_NORMAL); // Create a window for display.
    imshow(window_name, img);                // Show our image inside it.

    waitKey(0); // Wait for a keystroke in the window

    destroyWindow(window_name);
}

// reference: https://www.learnopencv.com/applycolormap-for-pseudocoloring-in-opencv-c-python/
void FastPlot::ShowMatrixAsColorMap(const Eigen::MatrixXd &matrix, std::string window_name)
{
    cv::Mat grey_img, color_img;

    // scale matrix to 0-255
    Eigen::MatrixXd scaled_matrix = (matrix + Eigen::MatrixXd::Ones(matrix.rows(), matrix.cols()) * matrix.minCoeff()) / (matrix.maxCoeff() - matrix.minCoeff()) * 255.0;
    eigen2cv(scaled_matrix, grey_img);

    grey_img.convertTo(grey_img, CV_8U);
    applyColorMap(grey_img, color_img, COLORMAP_JET);

    namedWindow(window_name, WINDOW_NORMAL); // Create a window for display.
    imshow(window_name, color_img);           // Show our image inside it.

    waitKey(0); // Wait for a keystroke in the window

    destroyWindow(window_name);
}