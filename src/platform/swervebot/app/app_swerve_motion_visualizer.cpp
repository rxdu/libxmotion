/*
 * @file app_swerve_motion_visualizer.cpp
 * @date 11/14/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>

#include "swerve_motion_visualizer.hpp"

using namespace xmotion;

int main(int argc, char* argv[]) {
  SwerveMotionVisualizer visualizer;
  if (!visualizer.Initialize()) {
    std::cout << "Failed to initialize visualizer" << std::endl;
    return -1;
  }
  visualizer.Run();

  return 0;
}