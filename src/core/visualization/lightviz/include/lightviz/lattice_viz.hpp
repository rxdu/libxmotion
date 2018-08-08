/* 
 * lattice_viz.hpp
 * 
 * Created on: Aug 08, 2018 02:41
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LATTICE_VIZ_HPP
#define LATTICE_VIZ_HPP

#include <Eigen/Dense>

namespace librav
{
namespace LightViz
{
void ShowPathOnMatrixAsColorMap(const Eigen::MatrixXd &matrix, std::string window_name = "Matrix Color Map", bool save_img = false);
}
} // namespace librav

#endif /* LATTICE_VIZ_HPP */
