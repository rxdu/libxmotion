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

#include "fastplot/cvplot_utils.hpp"

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

// Reference:
//  [1] https://www.learnopencv.com/applycolormap-for-pseudocoloring-in-opencv-c-python/
cv::Mat FastPlot::CreateColorMapFromEigenMatrix(const Eigen::MatrixXd &matrix)
{
    cv::Mat grey_img, color_img;

    // scale matrix to 0-255
    Eigen::MatrixXd scaled_matrix = (matrix + Eigen::MatrixXd::Ones(matrix.rows(), matrix.cols()) * matrix.minCoeff()) / (matrix.maxCoeff() - matrix.minCoeff()) * 255.0;
    // internally transposed for consistency of indexing so transpose back here
    Eigen::MatrixXd rotated_matrix = scaled_matrix.transpose();
    eigen2cv(rotated_matrix, grey_img);

    grey_img.convertTo(grey_img, CV_8U);
    applyColorMap(grey_img, color_img, COLORMAP_JET);

    return color_img;
}

void FastPlot::ShowImage(cv::Mat img, std::string window_name, bool save_img)
{
    namedWindow(window_name, WINDOW_NORMAL); // Create a window for display.
    imshow(window_name, img);                // Show our image inside it.

    waitKey(0); // Wait for a keystroke in the window

    if (save_img)
        imwrite(window_name + ".jpg", img);

    destroyWindow(window_name);
}

void FastPlot::ShowMatrixAsImage(const Eigen::MatrixXd &matrix, std::string window_name, bool save_img)
{
    cv::Mat img;
    eigen2cv(matrix, img);
    namedWindow(window_name, WINDOW_NORMAL); // Create a window for display.
    imshow(window_name, img);                // Show our image inside it.

    waitKey(0); // Wait for a keystroke in the window

    if (save_img)
        imwrite(window_name + ".jpg", img);

    destroyWindow(window_name);
}

void FastPlot::ShowMatrixAsColorMap(const Eigen::MatrixXd &matrix, std::string window_name, bool save_img)
{
    cv::Mat color_img = CreateColorMapFromEigenMatrix(matrix);

    cv::flip(color_img, color_img, 0);

    namedWindow(window_name, WINDOW_NORMAL); // Create a window for display.
    imshow(window_name, color_img);          // Show our image inside it.

    waitKey(0); // Wait for a keystroke in the window

    if (save_img)
        imwrite(window_name + ".jpg", color_img);

    destroyWindow(window_name);
}

void FastPlot::ShowPathOnColorMap(const Eigen::MatrixXd &matrix, const Eigen::MatrixXi &path, std::string window_name, bool save_img)
{
    cv::Mat color_img = CreateColorMapFromEigenMatrix(matrix);

    // add path to image
    for (int32_t r = 0; r < path.rows() - 1; ++r)
        PlotUtils::DrawLine(color_img, cv::Point(path(r, 0), path(r, 1)), cv::Point(path(r + 1, 0), path(r + 1, 1)), FastplotColors::ln_color);
    // PlotUtils::DrawPoint(color_img, cv::Point(path(0, 0), path(0, 1)), FastplotColors::start_color);
    // PlotUtils::DrawPoint(color_img, cv::Point(path(path.rows() - 1, 0), path(path.rows() - 1, 1)), FastplotColors::finish_color);
    color_img.at<cv::Vec3b>(path(0, 1), path(0, 0))[0] = FastplotColors::start_color(0);
    color_img.at<cv::Vec3b>(path(0, 1), path(0, 0))[1] = FastplotColors::start_color(1);
    color_img.at<cv::Vec3b>(path(0, 1), path(0, 0))[2] = FastplotColors::start_color(2);
    color_img.at<cv::Vec3b>(path(path.rows() - 1, 1), path(path.rows() - 1, 0))[0] = FastplotColors::finish_color(0);
    color_img.at<cv::Vec3b>(path(path.rows() - 1, 1), path(path.rows() - 1, 0))[1] = FastplotColors::finish_color(1);
    color_img.at<cv::Vec3b>(path(path.rows() - 1, 1), path(path.rows() - 1, 0))[2] = FastplotColors::finish_color(2);

    cv::flip(color_img, color_img, 0);

    namedWindow(window_name, WINDOW_NORMAL); // Create a window for display.
    imshow(window_name, color_img);          // Show our image inside it.

    waitKey(0); // Wait for a keystroke in the window

    if (save_img)
        imwrite(window_name + ".jpg", color_img);

    destroyWindow(window_name);
}