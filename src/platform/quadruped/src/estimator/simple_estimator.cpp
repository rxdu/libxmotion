/*
 * @file simple_estimator.cpp
 * @date 7/17/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "quadruped/estimator/simple_estimator.hpp"

#include "quadruped/robot_model/quadruped_model.hpp"
#include "quadruped/matrix_helper.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
namespace {
/**
 * @brief Calculate foot position wrt body in world frame p_sfB
 * @param robot_model
 * @param q
 * @param orientation
 * @return
 */
std::array<Position3d, 4> GetFootToBodyPositionWrtWorld(
    std::shared_ptr<QuadrupedModel> robot_model,
    const QuadrupedModel::AllJointVar& q, const Quaterniond& orientation) {
  std::array<Position3d, 4> foot_pos;
  for (int i = 0; i < 4; ++i) {
    auto index = static_cast<LegIndex>(i);
    foot_pos[i] = robot_model->GetFootPosition(index, q.segment<3>(i * 3),
                                               QuadrupedModel::RefFrame::kBase);
    foot_pos[i] = orientation.toRotationMatrix() * foot_pos[i];
  }
  return foot_pos;
}

/**
 * @brief Calculate foot velocity wrt body in world frame v_sfB
 * @param robot_model
 * @param q
 * @param q_dot
 * @param orientation
 * @param gyro
 * @return
 */
std::array<Velocity3d, 4> GetFootToBodyVelocityWrtWorld(
    std::shared_ptr<QuadrupedModel> robot_model,
    const QuadrupedModel::AllJointVar& q,
    const QuadrupedModel::AllJointVar& q_dot, const Quaterniond& orientation,
    const Eigen::Vector3d& gyro) {
  std::array<Velocity3d, 4> foot_vel;
  for (int i = 0; i < 4; ++i) {
    auto index = static_cast<LegIndex>(i);
    foot_vel[i] = robot_model->GetFootVelocity(index, q.segment<3>(i * 3),
                                               q_dot.segment<3>(i * 3));
    foot_vel[i] +=
        MatrixHelper::GetSkewSymmetricMatrix(gyro) *
        robot_model->GetFootPosition(index, q.segment<3>(i * 3),
                                     QuadrupedModel::RefFrame::kBase);
    foot_vel[i] = orientation.toRotationMatrix() * foot_vel[i];
  }
  return foot_vel;
}

// from mathTools.h in unitree_guide
template <typename T>
inline T WindowFunc(const T x, const T windowRatio, const T xRange = 1.0,
                    const T yRange = 1.0) {
  if ((x < 0) || (x > xRange)) {
    XLOG_ERROR_STREAM("[ERROR][windowFunc] The x="
                      << x << ", which should between [0, xRange]");
  }
  if ((windowRatio <= 0) || (windowRatio >= 0.5)) {
    XLOG_ERROR_STREAM("[ERROR][windowFunc] The windowRatio="
                      << windowRatio << ", which should between [0, 0.5]");
  }

  if (x / xRange < windowRatio) {
    return x * yRange / (xRange * windowRatio);
  } else if (x / xRange > 1 - windowRatio) {
    return yRange * (xRange - x) / (xRange * windowRatio);
  } else {
    return yRange;
  }
}
}  // namespace

////////////////////////////////////////////////////////////////////////////////

SimpleEstimator::SimpleEstimator(const EstimatorSettings& settings,
                                 std::shared_ptr<QuadrupedModel> robot_model)
    : settings_(settings), robot_model_(robot_model) {}

void SimpleEstimator::Initialize(const SimpleEstimator::Params& params) {
  params_ = params;
  x_hat_ = params_.x0;

  // TODO (rdu): temporary implementation, need to be updated
  /****************************************************************************/
  //  P_ = params_.p0;
  //  R_ = params_.r0;
  P_.setIdentity();
  P_ *= large_variance;
  R_ << 0.008, 0.012, -0.000, -0.009, 0.012, 0.000, 0.009, -0.009, -0.000,
      -0.009, -0.009, 0.000, -0.000, 0.000, -0.000, 0.000, -0.000, -0.001,
      -0.002, 0.000, -0.000, -0.003, -0.000, -0.001, 0.000, 0.000, 0.000, 0.000,
      0.012, 0.019, -0.001, -0.014, 0.018, -0.000, 0.014, -0.013, -0.000,
      -0.014, -0.014, 0.001, -0.001, 0.001, -0.001, 0.000, 0.000, -0.001,
      -0.003, 0.000, -0.001, -0.004, -0.000, -0.001, 0.000, 0.000, 0.000, 0.000,
      -0.000, -0.001, 0.001, 0.001, -0.001, 0.000, -0.000, 0.000, -0.000, 0.001,
      0.000, -0.000, 0.000, -0.000, 0.000, 0.000, -0.000, -0.000, 0.000, -0.000,
      -0.000, -0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, -0.009, -0.014,
      0.001, 0.010, -0.013, 0.000, -0.010, 0.010, 0.000, 0.010, 0.010, -0.000,
      0.001, 0.000, 0.000, 0.001, -0.000, 0.001, 0.002, -0.000, 0.000, 0.003,
      0.000, 0.001, 0.000, 0.000, 0.000, 0.000, 0.012, 0.018, -0.001, -0.013,
      0.018, -0.000, 0.013, -0.013, -0.000, -0.013, -0.013, 0.001, -0.001,
      0.000, -0.001, 0.000, 0.001, -0.001, -0.003, 0.000, -0.001, -0.004,
      -0.000, -0.001, 0.000, 0.000, 0.000, 0.000, 0.000, -0.000, 0.000, 0.000,
      -0.000, 0.001, 0.000, 0.000, -0.000, 0.000, 0.000, -0.000, -0.000, 0.000,
      -0.000, 0.000, 0.000, 0.000, -0.000, -0.000, -0.000, -0.000, 0.000, 0.000,
      0.000, 0.000, 0.000, 0.000, 0.009, 0.014, -0.000, -0.010, 0.013, 0.000,
      0.010, -0.010, -0.000, -0.010, -0.010, 0.000, -0.001, 0.000, -0.001,
      0.000, -0.000, -0.001, -0.001, 0.000, -0.000, -0.003, -0.000, -0.001,
      0.000, 0.000, 0.000, 0.000, -0.009, -0.013, 0.000, 0.010, -0.013, 0.000,
      -0.010, 0.009, 0.000, 0.010, 0.010, -0.000, 0.001, -0.000, 0.000, -0.000,
      0.000, 0.001, 0.002, 0.000, 0.000, 0.003, 0.000, 0.001, 0.000, 0.000,
      0.000, 0.000, -0.000, -0.000, -0.000, 0.000, -0.000, -0.000, -0.000,
      0.000, 0.001, 0.000, 0.000, 0.000, 0.000, -0.000, 0.000, -0.000, 0.000,
      -0.000, 0.000, -0.000, 0.000, 0.000, -0.000, -0.000, 0.000, 0.000, 0.000,
      0.000, -0.009, -0.014, 0.001, 0.010, -0.013, 0.000, -0.010, 0.010, 0.000,
      0.010, 0.010, -0.000, 0.001, 0.000, 0.000, -0.000, -0.000, 0.001, 0.002,
      -0.000, 0.000, 0.003, 0.000, 0.001, 0.000, 0.000, 0.000, 0.000, -0.009,
      -0.014, 0.000, 0.010, -0.013, 0.000, -0.010, 0.010, 0.000, 0.010, 0.010,
      -0.000, 0.001, -0.000, 0.000, -0.000, 0.000, 0.001, 0.002, -0.000, 0.000,
      0.003, 0.001, 0.001, 0.000, 0.000, 0.000, 0.000, 0.000, 0.001, -0.000,
      -0.000, 0.001, -0.000, 0.000, -0.000, 0.000, -0.000, -0.000, 0.001, 0.000,
      -0.000, -0.000, -0.000, 0.000, 0.000, -0.000, 0.000, 0.000, 0.000, 0.000,
      0.000, 0.000, 0.000, 0.000, 0.000, -0.000, -0.001, 0.000, 0.001, -0.001,
      -0.000, -0.001, 0.001, 0.000, 0.001, 0.001, 0.000, 1.708, 0.048, 0.784,
      0.062, 0.042, 0.053, 0.077, 0.001, -0.061, 0.046, -0.019, -0.029, 0.000,
      0.000, 0.000, 0.000, 0.000, 0.001, -0.000, 0.000, 0.000, 0.000, 0.000,
      -0.000, -0.000, 0.000, -0.000, -0.000, 0.048, 5.001, -1.631, -0.036,
      0.144, 0.040, 0.036, 0.016, -0.051, -0.067, -0.024, -0.005, 0.000, 0.000,
      0.000, 0.000, -0.000, -0.001, 0.000, 0.000, -0.001, -0.000, -0.001, 0.000,
      0.000, 0.000, 0.000, -0.000, 0.784, -1.631, 1.242, 0.057, -0.037, 0.018,
      0.034, -0.017, -0.015, 0.058, -0.021, -0.029, 0.000, 0.000, 0.000, 0.000,
      0.000, 0.000, 0.000, 0.001, 0.000, 0.000, 0.000, -0.000, -0.000, -0.000,
      -0.000, -0.000, 0.062, -0.036, 0.057, 6.228, -0.014, 0.932, 0.059, 0.053,
      -0.069, 0.148, 0.015, -0.031, 0.000, 0.000, 0.000, 0.000, -0.000, 0.000,
      -0.000, -0.000, 0.001, 0.000, -0.000, 0.000, 0.000, -0.000, 0.000, 0.000,
      0.042, 0.144, -0.037, -0.014, 3.011, 0.986, 0.076, 0.030, -0.052, -0.027,
      0.057, 0.051, 0.000, 0.000, 0.000, 0.000, -0.001, -0.001, -0.000, 0.001,
      -0.001, 0.000, -0.001, 0.001, -0.000, 0.001, 0.001, 0.000, 0.053, 0.040,
      0.018, 0.932, 0.986, 0.885, 0.090, 0.044, -0.055, 0.057, 0.051, -0.003,
      0.000, 0.000, 0.000, 0.000, -0.002, -0.003, 0.000, 0.002, -0.003, -0.000,
      -0.001, 0.002, 0.000, 0.002, 0.002, -0.000, 0.077, 0.036, 0.034, 0.059,
      0.076, 0.090, 6.230, 0.139, 0.763, 0.013, -0.019, -0.024, 0.000, 0.000,
      0.000, 0.000, 0.000, 0.000, -0.000, -0.000, 0.000, -0.000, 0.000, 0.000,
      -0.000, -0.000, -0.000, 0.000, 0.001, 0.016, -0.017, 0.053, 0.030, 0.044,
      0.139, 3.130, -1.128, -0.010, 0.131, 0.018, 0.000, 0.000, 0.000, 0.000,
      -0.000, -0.001, -0.000, 0.000, -0.001, -0.000, -0.000, 0.000, 0.000,
      0.000, 0.000, 0.000, -0.061, -0.051, -0.015, -0.069, -0.052, -0.055,
      0.763, -1.128, 0.866, -0.022, -0.053, 0.007, 0.000, 0.000, 0.000, 0.000,
      -0.003, -0.004, -0.000, 0.003, -0.004, -0.000, -0.003, 0.003, 0.000,
      0.003, 0.003, 0.000, 0.046, -0.067, 0.058, 0.148, -0.027, 0.057, 0.013,
      -0.010, -0.022, 2.437, -0.102, 0.938, 0.000, 0.000, 0.000, 0.000, -0.000,
      -0.000, 0.000, 0.000, -0.000, 0.000, -0.000, 0.000, -0.000, 0.000, 0.001,
      0.000, -0.019, -0.024, -0.021, 0.015, 0.057, 0.051, -0.019, 0.131, -0.053,
      -0.102, 4.944, 1.724, 0.000, 0.000, 0.000, 0.000, -0.001, -0.001, 0.000,
      0.001, -0.001, 0.000, -0.001, 0.001, -0.000, 0.001, 0.001, 0.000, -0.029,
      -0.005, -0.029, -0.031, 0.051, -0.003, -0.024, 0.018, 0.007, 0.938, 1.724,
      1.569, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
      0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
      0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0, 0.000,
      0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
      0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
      0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0, 0.000, 0.000, 0.000,
      0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
      0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
      0.000, 0.000, 0.000, 0.000, 0.000, 1.0, 0.000, 0.000, 0.000, 0.000, 0.000,
      0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
      0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
      0.000, 0.000, 0.000, 1.0;
  /****************************************************************************/

  Q_ = settings_.simple_estimator.Q_diag.asDiagonal();

  // A = I + dt * A_c, equation 7.14, 7.18
  A_ = Eigen::Matrix<double, 18, 18>::Identity();
  A_.block<3, 3>(0, 3) =
      Eigen::Matrix<double, 3, 3>::Identity() * settings_.expected_dt;

  // B = dt * B_c, equation 7.14, 7.18
  B_ = Eigen::Matrix<double, 18, 3>::Zero();
  B_.block<3, 3>(3, 0) =
      Eigen::Matrix<double, 3, 3>::Identity() * settings_.expected_dt;

  // equation 7.14
  C_ = Eigen::Matrix<double, 28, 18>::Zero();
  C_.block<3, 3>(0, 0) = -Eigen::Matrix<double, 3, 3>::Identity();
  C_.block<3, 3>(3, 0) = -Eigen::Matrix<double, 3, 3>::Identity();
  C_.block<3, 3>(6, 0) = -Eigen::Matrix<double, 3, 3>::Identity();
  C_.block<3, 3>(9, 0) = -Eigen::Matrix<double, 3, 3>::Identity();
  C_.block<3, 3>(12, 3) = -Eigen::Matrix<double, 3, 3>::Identity();
  C_.block<3, 3>(15, 3) = -Eigen::Matrix<double, 3, 3>::Identity();
  C_.block<3, 3>(18, 3) = -Eigen::Matrix<double, 3, 3>::Identity();
  C_.block<3, 3>(21, 3) = -Eigen::Matrix<double, 3, 3>::Identity();
  C_.block<3, 3>(0, 6) = Eigen::Matrix<double, 3, 3>::Identity();
  C_.block<12, 12>(0, 6) = Eigen::Matrix<double, 12, 12>::Identity();
  C_(24, 8) = 1;
  C_(25, 11) = 1;
  C_(26, 14) = 1;
  C_(27, 17) = 1;

  // save initial P and R
  Q_0_ = Q_;
  R_0_ = R_;
}

void SimpleEstimator::Update(const QuadrupedModel::SensorData& sensor_data,
                             double dt) {
//  XLOG_INFO("SimpleEstimator::Update, dt {}", dt);
//  if (std::abs(dt - settings_.expected_dt) / settings_.expected_dt > 0.1) {
//    XLOG_WARN("SimpleEstimator::Update, dt {} is not expected as setting {}",
//              dt, settings_.expected_dt);
//  }

  foot_height.setZero();
  std::array<Position3d, 4> foot_pos = GetFootToBodyPositionWrtWorld(
      robot_model_, sensor_data.q, sensor_data.quaternion);
  std::array<Velocity3d, 4> foot_vel = GetFootToBodyVelocityWrtWorld(
      robot_model_, sensor_data.q, sensor_data.q_dot, sensor_data.quaternion,
      sensor_data.gyroscope);

  Q_ = Q_0_;
  R_ = R_0_;
  auto contact = robot_model_->GetFootContactState();
  for (int i = 0; i < 4; ++i) {
    if (contact(i) == 1) {
      Q_.block<3, 3>(6 + 3 * i, 6 + 3 * i) =
          large_variance * Eigen::Matrix<double, 3, 3>::Identity();
      R_.block<3, 3>(12 + 3 * i, 12 + 3 * i) =
          large_variance * Eigen::Matrix<double, 3, 3>::Identity();
      R_(24 + i, 24 + i) = large_variance;
    } else {
      //      double belief = WindowFunc()
    }
  }
}
}  // namespace xmotion