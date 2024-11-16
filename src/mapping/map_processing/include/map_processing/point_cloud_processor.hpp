/*
 * @file point_cloud_processor.hpp
 * @date 9/6/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_POINT_CLOUD_PROCESSOR_HPP
#define XMOTION_POINT_CLOUD_PROCESSOR_HPP

#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "interface/type/geometry_types.hpp"

namespace xmotion {
class PointCloudProcessor {
 public:
  PointCloudProcessor() = default;
  ~PointCloudProcessor() = default;

  bool LoadData(const std::string &pc_file);
  void SaveData(const std::string &pc_file);
  void SaveData(const std::string &pc_file,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

  void CropAlongZAxis(double z_min, double z_max);

  pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud() { return cloud_; }

 private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
};
}  // namespace xmotion

#endif  // XMOTION_POINT_CLOUD_PROCESSOR_HPP