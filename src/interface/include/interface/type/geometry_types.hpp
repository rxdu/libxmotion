/*
 * geometry_types.hpp
 *
 * Created on 7/7/24 3:18 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_GEOMETRY_TYPES_HPP
#define QUADRUPED_MOTION_GEOMETRY_TYPES_HPP

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

namespace xmotion {
// workspace Cartesian coordinates
using Position3d = Eigen::Vector3d;
using Quaterniond = Eigen::Quaterniond;
using RotMatrix3d = Eigen::Matrix<double, 3, 3>;
using HomoMatrix3d = Eigen::Matrix<double, 4, 4>;

using Velocity3d = Eigen::Vector3d;
using Acceleration3d = Eigen::Vector3d;

// joint space coordinates
using JointPosition3d = Eigen::Vector3d;
using JointVelocity3d = Eigen::Vector3d;

// force/torque
using Force3d = Eigen::Vector3d;
using Torque3d = Eigen::Vector3d;
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_GEOMETRY_TYPES_HPP
