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
  std::string pc_file;

  if (argc > 1) {
    pc_file = argv[1];
  } else {
    std::cout << "Usage: " << argv[0] << " <point_cloud_file>" << std::endl;
    return -1;
  }

  PointCloudProcessor pcp;
  if (pcp.LoadData(pc_file)) {
    std::cout << "Point cloud loaded successfully" << std::endl;
  } else {
    std::cout << "Failed to load point cloud" << std::endl;
  }
  auto cloud = pcp.GetCloud();
  std::cout << "number of points: " << cloud->size() << std::endl;

  //////////////////////////////////////////////////////////////////
  // post processing
  pcp.CropAlongZAxis(-0.3, 1.2);

  cloud = pcp.GetCloud();
  std::cout << "number of points: " << cloud->size() << std::endl;

  pcp.SaveData("cropped.pcd");

  //  RotMatrix3d rot_mat;
//  rot_mat << 0.999959, -0.000240776, -0.319849, -0.0002407, 0.999644, -0.947049,
//      -0.00901282, -0.0266947, 0.0281772;
//  rot_mat.normalize();
//  std::cout << "rotation matrix: " << std::endl << rot_mat << std::endl;
//  std::cout << "norm: " << rot_mat.norm() << std::endl;
//
//  // apply transformation
//  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(
//      new pcl::PointCloud<pcl::PointXYZ>);
//  for (const auto& point : *cloud) {
//    Eigen::Vector3d pt(point.x, point.y, point.z);
//    Eigen::Vector3d new_pt = rot_mat * pt;
//    transformed_cloud->push_back(
//        pcl::PointXYZ(new_pt(0), new_pt(1), new_pt(2)));
//  }

  //////////////////////////////////////////////////////////////////

  // visualize the point cloud
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