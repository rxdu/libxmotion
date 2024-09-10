/*
 * @file pointcloud_processor.cpp
 * @date 9/6/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "map_processing/point_cloud_processor.hpp"

#include "logging/xlogger.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

namespace xmotion {
bool PointCloudProcessor::LoadData(const std::string& pc_file) {
  if (pc_file.empty()) {
    XLOG_ERROR("Point cloud file path is empty");
    return false;
  }
  auto suffix = pc_file.substr(pc_file.find_last_of('.') + 1);
  if (suffix == "pcd") {
    XLOG_INFO("Loading PCD file");
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  } else if (suffix == "ply") {
    XLOG_INFO("Loading PLY file");
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(pc_file, *cloud_) == -1) {
      XLOG_ERROR("Failed to load PLY file");
      return false;
    }
  } else {
    XLOG_ERROR("Unsupported point cloud file format: {}", suffix);
    return false;
  }
  return true;
}

void PointCloudProcessor::SaveData(const std::string& pc_file) {
  SaveData(pc_file, cloud_);
}

void PointCloudProcessor::SaveData(
    const std::string& pc_file,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  if (cloud_ == nullptr) {
    XLOG_ERROR("Point cloud is empty, nothing to save");
    return;
  }
  if (pc_file.empty()) {
    XLOG_ERROR("Point cloud file path is empty");
    return;
  }

  // save to pcd file to current folder
  if (pcl::io::savePCDFile(pc_file, *cloud) == -1) {
    XLOG_ERROR("Failed to save PCD file");
  }
}

void PointCloudProcessor::CropAlongZAxis(double z_min, double z_max) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud;
  new_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& point : *cloud_) {
    if (point.z >= z_min && point.z <= z_max) {
      new_cloud->push_back(point);
    }
  }
  cloud_ = new_cloud;
}
}  // namespace xmotion