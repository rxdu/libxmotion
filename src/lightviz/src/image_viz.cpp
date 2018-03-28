/* 
 * image_viz.cpp
 * 
 * Created on: Mar 28, 2018 14:47
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "lightviz/image_viz.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace librav;
using namespace cv;

namespace
{
// Reference:
//  [1] https://www.learnopencv.com/applycolormap-for-pseudocoloring-in-opencv-c-python/
cv::Mat CreateColorMapFromEigenMatrix(const Eigen::MatrixXd &matrix)
{
    cv::Mat grey_img, color_img;

    // scale matrix to 0-255
    Eigen::MatrixXd scaled_matrix = (matrix + Eigen::MatrixXd::Ones(matrix.rows(), matrix.cols()) * matrix.minCoeff()) / (matrix.maxCoeff() - matrix.minCoeff()) * 255.0;
    cv::eigen2cv(scaled_matrix, grey_img);

    grey_img.convertTo(grey_img, CV_8U);
    cv::applyColorMap(grey_img, color_img, COLORMAP_JET);

    return color_img;
}
}

void LightViz::ShowImage(cv::Mat img, std::string window_name, bool save_img)
{
    namedWindow(window_name, WINDOW_NORMAL); // Create a window for display.
    imshow(window_name, img);                // Show our image inside it.

    waitKey(0); // Wait for a keystroke in the window

    if (save_img)
        imwrite(window_name + ".jpg", img);

    destroyWindow(window_name);
}

void LightViz::ShowImage(std::string file_name, std::string window_name)
{
    cv::Mat img = imread(file_name);
    ShowImage(img, window_name, false);
}

void LightViz::ShowMatrixAsImage(const Eigen::MatrixXd &matrix, std::string window_name, bool save_img)
{
    cv::Mat img;
    eigen2cv(matrix, img);
    img.convertTo(img, CV_8UC3);
    ShowImage(img, window_name, save_img);
}

void LightViz::ShowMatrixAsColorMap(const Eigen::MatrixXd &matrix, std::string window_name, bool save_img)
{
    cv::Mat color_img = CreateColorMapFromEigenMatrix(matrix);
    ShowImage(color_img, window_name, save_img);
}