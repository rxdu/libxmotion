/*
 * simple_point.hpp
 *
 * Created on: Aug 10, 2018 10:56
 * Description: a convenient point type for the CGAL wrappers
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SIMPLE_POINT_HPP
#define SIMPLE_POINT_HPP

#include <eigen3/Eigen/Dense>

namespace xmotion {
using SimplePoint2 = Eigen::Vector2d;
using SimplePoint3 = Eigen::Vector3d;
}  // namespace xmotion

#endif /* SIMPLE_POINT_HPP */
