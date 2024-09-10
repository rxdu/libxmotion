/*
 * @file test_pc_processor.cpp
 * @date 9/6/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <thread>

#include <pcl/visualization/pcl_visualizer.h>

#include "map_processing/point_cloud_processor.hpp"
#include "map_processing/trajectory_processor.hpp"

#include "cvdraw/cvdraw.hpp"

using namespace xmotion;
using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
  std::string pc_file;
  std::string traj_file;

  if (argc > 2) {
    pc_file = argv[1];
    traj_file = argv[2];
  } else {
    std::cout << "Usage: " << argv[0] << " <point_cloud_file> <traj_file>"
              << std::endl;
    return -1;
  }

  // load pointcloud
  PointCloudProcessor pc_proc;
  if (pc_proc.LoadData(pc_file)) {
    std::cout << "Point cloud loaded successfully" << std::endl;
  } else {
    std::cout << "Failed to load point cloud" << std::endl;
  }
  auto cloud = pc_proc.GetCloud();
  std::cout << "number of points in pointcloud: " << cloud->size() << std::endl;

  // load trajectory
  xmotion::TrajectoryProcessor traj_proc;
  traj_proc.LoadData(traj_file);
  auto traj = traj_proc.GetTrajectory();
  std::cout << "number of points in trajectory: " << traj.points.size()
            << std::endl;
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
  if (angle > M_PI / 2.0) {
    angle = M_PI - angle;
  }
  std::cout << "angle between normal and z-axis: " << angle / M_PI * 180.0
            << std::endl;

  // apply rotation
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& point : *cloud) {
    Eigen::Vector3d pt(point.x, point.y, point.z);
    Eigen::Vector3d new_pt = rot_mat * pt;
    transformed_cloud->push_back(
        pcl::PointXYZ(new_pt(0), new_pt(1), new_pt(2)));
  }
  //  cloud = transformed_cloud;

  //  pc_proc.SaveData("cropped_transformed.pcd", cloud);

  ////////////////////////////////////////////////////////////////////////
  bool show_occupancy_map = false;
  bool show_point_cloud = false;

  // visualize the occupancy map
  if (show_occupancy_map) {
    CvCanvas canvas(8);
    canvas.Resize(-20, 80, -30, 120);
    canvas.SetMode(CvCanvas::DrawMode::GeometryInvertedY);
    // draw pointcloud
    {
      std::vector<cv::Point2d> pts;
      for (auto& pt : cloud->points) {
        pts.emplace_back(pt.x, pt.y);
        canvas.DrawPoint({pt.x, pt.y}, 1, CvColors::black_color, 1);
      }
    }
    // draw trajectory
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
  }

  if (show_point_cloud) {
    // visualize the point cloud
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    //    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    //    first_color(
    //        cloud, 0, 255, 0);
    //    viewer->addPointCloud<pcl::PointXYZ>(cloud, first_color, "sample
    //    cloud");

    //    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    //        second_color(transformed_cloud, 0, 0, 255);
    //    viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud, second_color,
    //                                         "sample cloud2");

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>
        height_color(transformed_cloud, "z");
    viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud, height_color,
                                         "sample cloud2");

    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    viewer->spin();
  }

  return 0;
}