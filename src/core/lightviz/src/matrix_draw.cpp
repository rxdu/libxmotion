/* 
 * matrix_draw.cpp
 * 
 * Created on: Aug 10, 2018 09:34
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#include "lightviz/details/matrix_draw.hpp"

using namespace librav;

// Reference:
//  [1] https://www.learnopencv.com/applycolormap-for-pseudocoloring-in-opencv-c-python/
cv::Mat LightViz::CreateColorMapFromEigenMatrix(const Eigen::MatrixXd &matrix)
{
    cv::Mat grey_img, color_img;

    // scale matrix to 0-255
    Eigen::MatrixXd scaled_matrix = (matrix + Eigen::MatrixXd::Ones(matrix.rows(), matrix.cols()) * matrix.minCoeff()) / (matrix.maxCoeff() - matrix.minCoeff()) * 255.0;
    cv::eigen2cv(scaled_matrix, grey_img);

    grey_img.convertTo(grey_img, CV_8U);
    cv::applyColorMap(grey_img, color_img, cv::COLORMAP_JET);

    return color_img;
}