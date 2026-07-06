/*
 * test_mekf6.cpp
 *
 * MEKF6 correctness tests against synthetic IMU truth. Gravity observations
 * cannot observe yaw (rotation about gravity), so attitude assertions compare
 * gravity directions rather than full quaternions, and gyro-bias assertions
 * check the observable x/y components only.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <cmath>

#include <eigen3/Eigen/Eigenvalues>

#include "xmnav/estimation/mekf6.hpp"

using namespace xmotion;

namespace {

constexpr double kGravity = 9.81;

Mekf6::Params DefaultParams() {
  Mekf6::Params p;
  p.gravity_constant = kGravity;
  p.init_state_cov = Mekf6::StateCovariance::Identity() * 0.1;
  p.init_observation_noise_cov =
      Mekf6::ObservationNoiseCovariance::Identity() * 0.05;
  p.sigma_omega = Eigen::Vector3d::Constant(0.01);
  p.sigma_f = Eigen::Vector3d::Constant(0.05);
  p.sigma_beta_omega = Eigen::Vector3d::Constant(0.001);
  p.sigma_beta_f = Eigen::Vector3d::Constant(0.001);
  return p;
}

// accelerometer reading for a body at rest with attitude q (body-to-inertial)
Eigen::Vector3d GravityReading(const Eigen::Quaterniond& q_true) {
  return q_true.conjugate() * Eigen::Vector3d(0, 0, -kGravity);
}

// angle between the gravity directions predicted by two attitudes — the
// gravity-observable part of the attitude error
double GravityAngleError(const Eigen::Quaterniond& a,
                         const Eigen::Quaterniond& b) {
  Eigen::Vector3d ga = (a.conjugate() * Eigen::Vector3d(0, 0, -1)).normalized();
  Eigen::Vector3d gb = (b.conjugate() * Eigen::Vector3d(0, 0, -1)).normalized();
  return std::acos(std::min(1.0, std::max(-1.0, ga.dot(gb))));
}

}  // namespace

TEST(Mekf6Test, StaticAttitudeConvergence) {
  // true attitude: 10 deg roll, -7 deg pitch; filter starts at identity
  Eigen::Quaterniond q_true =
      Eigen::AngleAxisd(10.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(-7.0 * M_PI / 180.0, Eigen::Vector3d::UnitY());

  // With a single constant gravity vector at fixed attitude, attitude error
  // and accel bias are jointly unobservable (a constant residual fits both).
  // This scenario knows the accelerometer is clean, so it says so via a
  // tight accel-bias prior; motion is what separates the two in practice
  // (see TracksSlowRollRotation).
  Mekf6::Params params = DefaultParams();
  params.init_state_cov.block<3, 3>(12, 12) =
      Eigen::Matrix3d::Identity() * 1e-8;
  params.sigma_beta_f = Eigen::Vector3d::Constant(1e-6);

  Mekf6 mekf;
  mekf.Initialize(params);

  const double dt = 0.01;
  for (int i = 0; i < 6000; ++i) {
    ASSERT_TRUE(mekf.Update(Eigen::Vector3d::Zero(), GravityReading(q_true), dt));
  }

  EXPECT_LT(GravityAngleError(mekf.GetQuaternion(), q_true),
            0.5 * M_PI / 180.0);
}

TEST(Mekf6Test, GyroBiasRecovery) {
  // stationary and level; constant gyro bias on the observable axes
  const Eigen::Vector3d bias(0.02, -0.015, 0.0);
  Mekf6 mekf;
  mekf.Initialize(DefaultParams());

  const double dt = 0.01;
  const Eigen::Quaterniond q_true = Eigen::Quaterniond::Identity();
  for (int i = 0; i < 20000; ++i) {
    ASSERT_TRUE(mekf.Update(bias, GravityReading(q_true), dt));
  }

  // x/y gyro bias is observable through gravity; z (yaw drift) is not
  EXPECT_NEAR(mekf.GetGyroBias()(0), bias(0), 0.2 * std::abs(bias(0)));
  EXPECT_NEAR(mekf.GetGyroBias()(1), bias(1), 0.2 * std::abs(bias(1)));
  // and the attitude must stay level despite the bias
  EXPECT_LT(GravityAngleError(mekf.GetQuaternion(), q_true),
            1.0 * M_PI / 180.0);
}

TEST(Mekf6Test, TracksSlowRollRotation) {
  const double rate = 5.0 * M_PI / 180.0;  // 5 deg/s about body x
  Eigen::Quaterniond q_true = Eigen::Quaterniond::Identity();
  Mekf6 mekf;
  mekf.Initialize(DefaultParams());

  const double dt = 0.005;
  const Eigen::Vector3d omega(rate, 0, 0);
  double max_err = 0;
  for (int i = 0; i < 4000; ++i) {  // 20 s -> 100 deg of roll
    // integrate the truth with the same first-order rule
    q_true = Eigen::Quaterniond(
        q_true.coeffs() +
        0.5 * dt *
            (q_true * Eigen::Quaterniond(0, omega(0), omega(1), omega(2)))
                .coeffs());
    q_true.normalize();
    ASSERT_TRUE(mekf.Update(omega, GravityReading(q_true), dt));
    if (i > 200) {
      max_err = std::max(max_err,
                         GravityAngleError(mekf.GetQuaternion(), q_true));
    }
  }
  EXPECT_LT(max_err, 1.0 * M_PI / 180.0);
}

TEST(Mekf6Test, CovarianceStaysSymmetricPositive) {
  Mekf6 mekf;
  mekf.Initialize(DefaultParams());
  const Eigen::Quaterniond q_true = Eigen::Quaterniond::Identity();
  for (int i = 0; i < 5000; ++i) {
    ASSERT_TRUE(
        mekf.Update(Eigen::Vector3d::Zero(), GravityReading(q_true), 0.01));
  }
  const auto& P = mekf.GetCovariance();
  ASSERT_TRUE(P.allFinite());
  EXPECT_LT((P - P.transpose()).norm(), 1e-12);
  Eigen::SelfAdjointEigenSolver<Mekf6::StateCovariance> es(P);
  EXPECT_GT(es.eigenvalues().minCoeff(), -1e-12);
}

TEST(Mekf6Test, AccelGateRejectsDynamicAcceleration) {
  Mekf6 mekf;
  mekf.Initialize(DefaultParams());
  const Eigen::Quaterniond q_true = Eigen::Quaterniond::Identity();
  for (int i = 0; i < 500; ++i) {
    mekf.Update(Eigen::Vector3d::Zero(), GravityReading(q_true), 0.01);
  }
  const Eigen::Quaterniond q_before = mekf.GetQuaternion();

  // a hard 5 m/s^2 lateral acceleration on top of gravity
  Eigen::Vector3d accel = GravityReading(q_true) + Eigen::Vector3d(5, 0, 0);
  for (int i = 0; i < 200; ++i) {
    ASSERT_TRUE(mekf.Update(Eigen::Vector3d::Zero(), accel, 0.01));
    EXPECT_FALSE(mekf.LastUpdateUsedObservation());
  }
  // the gate keeps the attitude from being dragged by the false "gravity"
  EXPECT_LT(GravityAngleError(mekf.GetQuaternion(), q_before),
            0.1 * M_PI / 180.0);
}

TEST(Mekf6Test, RejectsInvalidInput) {
  Mekf6 mekf;
  mekf.Initialize(DefaultParams());
  const Eigen::Quaterniond before = mekf.GetQuaternion();

  EXPECT_FALSE(mekf.Update(Eigen::Vector3d::Zero(),
                           GravityReading(Eigen::Quaterniond::Identity()), 0.0));
  EXPECT_FALSE(mekf.Update(Eigen::Vector3d(NAN, 0, 0),
                           GravityReading(Eigen::Quaterniond::Identity()), 0.01));
  EXPECT_TRUE(before.coeffs().isApprox(mekf.GetQuaternion().coeffs()));
}
