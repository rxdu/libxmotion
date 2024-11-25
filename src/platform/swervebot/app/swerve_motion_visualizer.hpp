/*
 * @file swerve_motion_visualizer.hpp
 * @date 11/14/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SWERVE_MOTION_VISUALIZER_HPP
#define XMOTION_SWERVE_MOTION_VISUALIZER_HPP

#include <memory>

#include "imview/viewer.hpp"

namespace xmotion {
class SwerveMotionVisualizer {
 public:
  SwerveMotionVisualizer();
  ~SwerveMotionVisualizer() = default;

  // Run the application
  bool Initialize();
  void Run();

 private:
  std::unique_ptr<quickviz::Viewer> viewer_;
};
}  // namespace xmotion

#endif  // XMOTION_SWERVE_MOTION_VISUALIZER_HPP