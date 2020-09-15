/* 
 * matrix_draw.cpp
 * 
 * Created on: Aug 10, 2018 09:34
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "coreviz/matrix_draw.hpp"

#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace librav
{
// Reference:
//  [1] https://www.learnopencv.com/applycolormap-for-pseudocoloring-in-opencv-c-python/
cv::Mat MatrixViz::CreateColorMapFromEigenMatrix(const Eigen::MatrixXd &matrix, bool invert_y)
{
    cv::Mat grey_img, color_img;

    // scale matrix to 0-255
    Eigen::MatrixXd scaled_matrix = (matrix - Eigen::MatrixXd::Ones(matrix.rows(), matrix.cols()) * matrix.minCoeff()) / (matrix.maxCoeff() - matrix.minCoeff()) * 255.0;

    Eigen::MatrixXd inverted_matrix = Eigen::MatrixXd::Zero(matrix.rows(), matrix.cols());
    if (invert_y)
    {
        std::size_t row_num = matrix.rows();
        for (std::size_t i = 0; i < row_num; ++i)
            inverted_matrix.row(i) = scaled_matrix.row(row_num - i - 1);
        cv::eigen2cv(inverted_matrix, grey_img);
    }
    else
    {
        cv::eigen2cv(scaled_matrix, grey_img);
    }

    grey_img.convertTo(grey_img, CV_8U);
    cv::applyColorMap(grey_img, color_img, cv::COLORMAP_JET);

    return color_img;
}
}