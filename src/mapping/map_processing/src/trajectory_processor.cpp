/*
 * @file trajectory_processor.cpp
 * @date 9/6/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "map_processing/trajectory_processor.hpp"

#include <Eigen/Dense>
#include "rapidcsv.h"

#include "logging/xlogger.hpp"

namespace xmotion {
void TrajectoryProcessor::LoadData(const std::string& filename,
                                   TrajectoryProcessor::DataFormat format) {
  if (format == DataFormat::kTimePxPyPzQxQyQzQw) {
    rapidcsv::Document doc(filename, rapidcsv::LabelParams(-1, -1),
                           rapidcsv::SeparatorParams(' '));
    trajectory_.points.clear();
    for (int i = 0; i < doc.GetRowCount(); i++) {
      TrajectoryPoint3d point;
      point.time = doc.GetCell<double>(0, i);
      point.position.x() = doc.GetCell<double>(1, i);
      point.position.y() = doc.GetCell<double>(2, i);
      point.position.z() = doc.GetCell<double>(3, i);
      point.orientation.x() = doc.GetCell<double>(4, i);
      point.orientation.y() = doc.GetCell<double>(5, i);
      point.orientation.z() = doc.GetCell<double>(6, i);
      point.orientation.w() = doc.GetCell<double>(7, i);
      //    XLOG_INFO("Time: {}, Position: {},{},{}, Orientation: {},{},{},{}",
      //              point.time, point.position.x(), point.position.y(),
      //              point.position.z(), point.orientation.coeffs().x(),
      //              point.orientation.coeffs().y(),
      //              point.orientation.coeffs().z(),
      //              point.orientation.coeffs().w());
      trajectory_.points.push_back(point);
    }
  } else {
    XLOG_ERROR("Unsupported data format");
  }
}

bool TrajectoryProcessor::FindPlane(uint32_t traj_idx_start,
                                    uint32_t traj_idx_end,
                                    PlaneDescriptor& plane) {
  if (traj_idx_start >= trajectory_.points.size() ||
      traj_idx_end >= trajectory_.points.size()) {
    XLOG_ERROR("Trajectory index out of range");
    return false;
  }

  // find a plane that fits the trajectory points
  size_t num_points = traj_idx_end - traj_idx_start + 1;
  //  Eigen::MatrixXd A(num_points, 4);
  //  Eigen::VectorXd b = Eigen::VectorXd::Zero(num_points);
  //  for (uint32_t i = traj_idx_start; i <= traj_idx_end; ++i) {
  //    A(i - traj_idx_start, 0) = trajectory_.points[i].position.x();
  //    A(i - traj_idx_start, 1) = trajectory_.points[i].position.y();
  //    A(i - traj_idx_start, 2) = trajectory_.points[i].position.z();
  //    A(i - traj_idx_start, 3) = 1.0;
  //    std::cout << "adding point: " << trajectory_.points[i].position.x() <<
  //    ","
  //              << trajectory_.points[i].position.y() << ","
  //              << trajectory_.points[i].position.z() << std::endl;
  //  }
  //  Eigen::VectorXd X = A.colPivHouseholderQr().solve(b);
  //  plane.a = X(0);
  //  plane.b = X(1);
  //  plane.c = X(2);
  //  plane.d = X(3);

  Eigen::Vector3d centroid(0, 0, 0);
  for (uint32_t i = traj_idx_start; i <= traj_idx_end; ++i) {
    centroid += Eigen::Vector3d(trajectory_.points[i].position.x(),
                                trajectory_.points[i].position.y(),
                                trajectory_.points[i].position.z());
  }
  centroid /= num_points;

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (uint32_t i = traj_idx_start; i <= traj_idx_end; ++i) {
    Eigen::Vector3d point(trajectory_.points[i].position.x(),
                          trajectory_.points[i].position.y(),
                          trajectory_.points[i].position.z());
    Eigen::Vector3d centered_point = point - centroid;
    covariance += centered_point * centered_point.transpose();
  }
  covariance /= num_points;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  Eigen::Vector3d normal = solver.eigenvectors().col(0);
  plane.a = normal.x();
  plane.b = normal.y();
  plane.c = normal.z();
  plane.d = -normal.dot(centroid);

  return true;
}

RotMatrix3d TrajectoryProcessor::ComputeTransformation(
    const PlaneDescriptor& plane) {
  Eigen::Vector3d normal(plane.a, plane.b, plane.c);
  normal.normalize();

  Eigen::Vector3d target_normal(0.0, 0.0, 1.0);
  Eigen::Vector3d rotation_axis = normal.cross(target_normal);

  double dot_product = normal.dot(target_normal);
  double angle = std::acos(dot_product);

  Eigen::Matrix3d skew_symmetric;
  skew_symmetric << 0, -rotation_axis.z(), rotation_axis.y(), rotation_axis.z(),
      0, -rotation_axis.x(), -rotation_axis.y(), rotation_axis.x(), 0;

  Eigen::Matrix3d rot_matrix =
      Eigen::Matrix3d::Identity() + std::sin(angle) * skew_symmetric +
      (1 - std::cos(angle)) * skew_symmetric * skew_symmetric;
  rot_matrix.normalize();

  return rot_matrix;
}
}  // namespace xmotion