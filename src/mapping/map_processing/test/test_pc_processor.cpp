/*
 * @file test_pc_processor.cpp
 * @date 9/6/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <thread>

#include "map_processing/point_cloud_processor.hpp"
#include <pcl/visualization/pcl_visualizer.h>

using namespace xmotion;
using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
  std::string pc_file =
      "/home/rdu/Workspace/weston_robot/wrdev_ws/ros2_ws/glim_slam/src/map/"
      "parking/parking_lot.ply";
  PointCloudProcessor pcp;

  if (argc > 1) {
    pc_file = argv[1];
  } else {
    std::cout << "Usage: " << argv[0] << " <point_cloud_file>" << std::endl;
    return -1;
  }

  if (pcp.LoadData(pc_file)) {
    std::cout << "Point cloud loaded successfully" << std::endl;
  } else {
    std::cout << "Failed to load point cloud" << std::endl;
  }
  auto cloud = pcp.GetCloud();
  std::cout << "number of points: " << cloud->size() << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  viewer->spin();

  return 0;
}