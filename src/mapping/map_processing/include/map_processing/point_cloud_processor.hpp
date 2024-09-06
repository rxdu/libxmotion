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

namespace xmotion {
class PointCloudProcessor {
 public:
  PointCloudProcessor() = default;
  ~PointCloudProcessor() = default;

  bool LoadData(const std::string &pc_file);

  pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud() { return cloud_; }

 private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
};
}  // namespace xmotion

#endif  // XMOTION_POINT_CLOUD_PROCESSOR_HPP