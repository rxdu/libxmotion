/*
 * @file test_trajectory_traj_proc.cpp
 * @date 9/6/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>

#include "map_processing/trajectory_processor.hpp"

#include "cvdraw/cvdraw.hpp"

using namespace xmotion;

int main(int argc, char* argv[]) {
  std::string csv_file =
      "/home/rdu/Workspace/weston_robot/wrdev_ws/ros2_ws/glim_slam/src/map/"
      "parking/traj_lidar.txt";

  if (argc > 1) {
    csv_file = argv[1];
  }

  xmotion::TrajectoryProcessor traj_proc;
  traj_proc.LoadData(csv_file);

  auto traj = traj_proc.GetTrajectory();
  std::cout << "number of points: " << traj.points.size() << std::endl;

  TrajectoryProcessor::PlaneDescriptor plane;
  if (traj_proc.FindPlane(0, traj.points.size() - 1, plane)) {
    std::cout << "plane: " << plane.a << "x + " << plane.b << "y + " << plane.c
              << "z + " << plane.d << " = 0" << std::endl;
  }

  RotMatrix3d rot_mat = traj_proc.ComputeTransformation(plane);
  std::cout << "rotation matrix: " << std::endl << rot_mat << std::endl;

  // calculate angle between normal and z-axis
  Eigen::Vector3d normal(plane.a, plane.b, plane.c);
  Eigen::Vector3d z_axis(0, 0, 1);
  double angle = std::acos(normal.dot(z_axis) / normal.norm());
  std::cout << "angle between normal and z-axis: " << angle / M_PI * 180.0
            << std::endl;

  //////////////////////////////////////////////////////////////////////////////
  // visualize trajectory
  CvCanvas canvas(10);
  canvas.Resize(-20, 80, -10, 90);
  canvas.SetMode(CvCanvas::DrawMode::GeometryInvertedY);
  {
    std::vector<cv::Point2d> pts;
    for (auto& pt : traj.points)
      pts.emplace_back(pt.position.x(), pt.position.y());
    for (std::size_t i = 0; i < pts.size() - 1; ++i)
      canvas.DrawLine({pts[i].x, pts[i].y}, {pts[i + 1].x, pts[i + 1].y},
                      CvColors::blue_color, 1);
  }
  canvas.DrawXYAxis();
  canvas.Show();

  return 0;
}