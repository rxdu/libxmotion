/* 
 * matrix_viz.hpp
 * 
 * Created on: Mar 28, 2018 21:02
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MATRIX_VIZ_HPP
#define MATRIX_VIZ_HPP

#include <Eigen/Dense>

namespace librav
{
namespace LightViz
{
void ShowMatrixAsImage(const Eigen::MatrixXd &matrix, std::string window_name = "Matrix Image", bool save_img = false);
void ShowMatrixAsColorMap(const Eigen::MatrixXd &matrix, std::string window_name = "Matrix Color Map", bool save_img = false);
}
}

#endif /* MATRIX_VIZ_HPP */
