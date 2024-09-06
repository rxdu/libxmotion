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
}  // namespace xmotion