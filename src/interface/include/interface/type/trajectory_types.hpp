/*
 * @file trajectory_types.hpp
 * @date 9/9/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_TRAJECTORY_TYPES_HPP
#define XMOTION_TRAJECTORY_TYPES_HPP

#include "interface/type/geometry_types.hpp"

namespace xmotion {
struct TrajectoryPoint3d {
  double time;
  Position3d position;
  Quaterniond orientation;

  Velocity3d velocity;
  Acceleration3d acceleration;
};

struct Trajectory3d {
  std::vector<TrajectoryPoint3d> points;
};
}  // namespace xmotion

#endif  // XMOTION_TRAJECTORY_TYPES_HPP
