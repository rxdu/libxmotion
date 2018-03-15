/* 
 * mathgl_utils.hpp
 * 
 * Created on: Mar 14, 2018 23:29
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef MATHGL_UTILS_HPP
#define MATHGL_UTILS_HPP

#include <iostream>

#include <Eigen/Dense>

#include "mgl2/mgl.h"
#include <mgl2/glut.h>

namespace librav
{
namespace MathGLUtils
{
  mglData ConvertTo2DMathGLData(const Eigen::MatrixXd &matrix);
}
}

#endif /* MATHGL_UTILS_HPP */
