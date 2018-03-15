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
template <typename Derivated>
mglData ConvertToMathGLData(const Eigen::DenseBase<Derivated> &matrix)
{
  const int32_t size_x = matrix.rows();
  const int32_t size_y = matrix.cols();

  mglData data(size_x, size_y);

  for (int32_t x = 0; x < size_x; ++x)
    for (int32_t y = 0; y < size_y; ++y)
      data.a[x + size_x * y] = matrix(x, y);

  return data;
}
}
}

#endif /* MATHGL_UTILS_HPP */
