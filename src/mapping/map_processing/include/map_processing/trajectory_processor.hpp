/*
 * @file trajectory_processor.hpp
 * @date 9/6/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_TRAJECTORY_PROCESSOR_HPP
#define XMOTION_TRAJECTORY_PROCESSOR_HPP

#include <string>

#include "interface/type/trajectory_types.hpp"

namespace xmotion {
class TrajectoryProcessor {
 public:
  enum class DataFormat { kTimePxPyPzQxQyQzQw = 0 };

  // plane: ax + by + cz + d = 0
  struct PlaneDescriptor {
    double a;
    double b;
    double c;
    double d;
  };

 public:
  TrajectoryProcessor() = default;

  void LoadData(const std::string& filename,
                DataFormat format = DataFormat::kTimePxPyPzQxQyQzQw);

  Trajectory3d GetTrajectory() const { return trajectory_; }

  bool FindPlane(uint32_t traj_idx_start, uint32_t traj_idx_end,
                 PlaneDescriptor& plane);
  RotMatrix3d ComputeTransformation(const PlaneDescriptor& plane);

 private:
  Trajectory3d trajectory_;
};
}  // namespace xmotion

#endif  // XMOTION_TRAJECTORY_PROCESSOR_HPP