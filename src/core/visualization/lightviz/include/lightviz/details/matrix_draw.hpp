/* 
 * matrix_draw.hpp
 * 
 * Created on: Aug 10, 2018 09:32
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef MATRIX_DRAW_HPP
#define MATRIX_DRAW_HPP

#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace librav
{
namespace LightViz
{
cv::Mat CreateColorMapFromEigenMatrix(const Eigen::MatrixXd &matrix);
}
}

#endif /* MATRIX_DRAW_HPP */
