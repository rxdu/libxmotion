/*
 * test_mekf9.cpp
 *
 * MEKF9 tests against synthetic 9-DOF truth. With two independent vector
 * observations (gravity + magnetic field), the full attitude — including
 * heading — and all three gyro-bias axes are observable, which is exactly
 * what these tests assert beyond the Mekf6 suite.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <cmath>

#include <eigen3/Eigen/Eigenvalues>

#include "xmnav/estimation/mekf9.hpp"

using namespace xmotion;

namespace {

constexpr double kGravity = 9.81;
const Eigen::Vector3d kMagRef(0.5, 0.0, -0.6);  // north component + dip

Mekf9::Params DefaultParams() {
  Mekf9::Params p;
  p.gravity_constant = kGravity;
  p.mag_reference = kMagRef;
  p.init_state_cov = Mekf9::StateCovariance::Identity() * 0.1;
  p.accel_noise_cov = Mekf9::ObservationNoiseCovariance::Identity() * 0.05;
  p.mag_noise_cov = Mekf9::ObservationNoiseCovariance::Identity() * 0.01;
  p.sigma_omega = Eigen::Vector3d::Constant(0.01);
  p.sigma_f = Eigen::Vector3d::Constant(0.05);
  p.sigma_beta_omega = Eigen::Vector3d::Constant(0.001);
  p.sigma_beta_f = Eigen::Vector3d::Constant(0.001);
  p.sigma_beta_m = Eigen::Vector3d::Constant(0.001);
  return p;
}

// sensor-clean scenarios state their knowledge as tight bias priors, which
// resolves the attitude/bias split of constant residuals
Mekf9::Params CleanSensorParams() {
  Mekf9::Params p = DefaultParams();
  p.init_state_cov.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 1e-8;
  p.init_state_cov.block<3, 3>(15, 15) = Eigen::Matrix3d::Identity() * 1e-8;
  p.sigma_beta_f = Eigen::Vector3d::Constant(1e-6);
  p.sigma_beta_m = Eigen::Vector3d::Constant(1e-6);
  return p;
}

Eigen::Vector3d GravityReading(const Eigen::Quaterniond& q_true) {
  return q_true.conjugate() * Eigen::Vector3d(0, 0, -kGravity);
}

Eigen::Vector3d MagReading(const Eigen::Quaterniond& q_true) {
  return q_true.conjugate() * kMagRef;
}

// full attitude error angle (heading included)
double FullAngleError(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
  double d = std::min(1.0, std::abs(a.coeffs().dot(b.coeffs())));
  return 2.0 * std::acos(d);
}

}  // namespace

TEST(Mekf9Test, FullAttitudeConvergenceIncludingYaw) {
  // 30 deg yaw + 10 deg roll — yaw is invisible to gravity, visible to mag
  Eigen::Quaterniond q_true =
      Eigen::AngleAxisd(30.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(10.0 * M_PI / 180.0, Eigen::Vector3d::UnitX());

  Mekf9 mekf;
  mekf.Initialize(CleanSensorParams());

  for (int i = 0; i < 6000; ++i) {
    ASSERT_TRUE(mekf.Update(Eigen::Vector3d::Zero(), GravityReading(q_true),
                            MagReading(q_true), 0.01));
  }

  EXPECT_LT(FullAngleError(mekf.GetQuaternion(), q_true), 0.5 * M_PI / 180.0);
}

TEST(Mekf9Test, GyroBiasRecoveryAllAxes) {
  // z-axis gyro bias produces yaw drift — observable only through the mag
  const Eigen::Vector3d bias(0.02, -0.015, 0.01);
  const Eigen::Quaterniond q_true = Eigen::Quaterniond::Identity();

  Mekf9 mekf;
  mekf.Initialize(DefaultParams());

  for (int i = 0; i < 20000; ++i) {
    ASSERT_TRUE(mekf.Update(bias, GravityReading(q_true), MagReading(q_true),
                            0.01));
  }

  for (int a = 0; a < 3; ++a) {
    EXPECT_NEAR(mekf.GetGyroBias()(a), bias(a), 0.2 * std::abs(bias(a)))
        << "axis " << a;
  }
  EXPECT_LT(FullAngleError(mekf.GetQuaternion(), q_true), 1.0 * M_PI / 180.0);
}

TEST(Mekf9Test, MagGateRejectsInterference) {
  const Eigen::Quaterniond q_true = Eigen::Quaterniond::Identity();
  Mekf9 mekf;
  mekf.Initialize(DefaultParams());
  for (int i = 0; i < 1000; ++i) {
    mekf.Update(Eigen::Vector3d::Zero(), GravityReading(q_true),
                MagReading(q_true), 0.01);
  }
  const Eigen::Quaterniond q_before = mekf.GetQuaternion();

  // hard-iron style disturbance: field magnitude far off the reference
  const Eigen::Vector3d disturbed = MagReading(q_true) * 2.0;
  for (int i = 0; i < 500; ++i) {
    ASSERT_TRUE(mekf.Update(Eigen::Vector3d::Zero(), GravityReading(q_true),
                            disturbed, 0.01));
    EXPECT_FALSE(mekf.LastUpdateUsedMag());
    EXPECT_TRUE(mekf.LastUpdateUsedAccel());
  }
  EXPECT_LT(FullAngleError(mekf.GetQuaternion(), q_before),
            0.1 * M_PI / 180.0);
}

TEST(Mekf9Test, CovarianceStaysSymmetricPositive) {
  const Eigen::Quaterniond q_true = Eigen::Quaterniond::Identity();
  Mekf9 mekf;
  mekf.Initialize(DefaultParams());
  for (int i = 0; i < 5000; ++i) {
    ASSERT_TRUE(mekf.Update(Eigen::Vector3d::Zero(), GravityReading(q_true),
                            MagReading(q_true), 0.01));
  }
  const auto& P = mekf.GetCovariance();
  ASSERT_TRUE(P.allFinite());
  EXPECT_LT((P - P.transpose()).norm(), 1e-12);
  Eigen::SelfAdjointEigenSolver<Mekf9::StateCovariance> es(P);
  EXPECT_GT(es.eigenvalues().minCoeff(), -1e-12);
}

TEST(Mekf9Test, RejectsInvalidInput) {
  Mekf9 mekf;
  mekf.Initialize(DefaultParams());
  const Eigen::Quaterniond before = mekf.GetQuaternion();
  const Eigen::Quaterniond id = Eigen::Quaterniond::Identity();

  EXPECT_FALSE(mekf.Update(Eigen::Vector3d::Zero(), GravityReading(id),
                           MagReading(id), 0.0));
  EXPECT_FALSE(mekf.Update(Eigen::Vector3d::Zero(), GravityReading(id),
                           Eigen::Vector3d(NAN, 0, 0), 0.01));
  EXPECT_TRUE(before.coeffs().isApprox(mekf.GetQuaternion().coeffs()));
}

// Regression: a diverged inertial-bias estimate must not lock its own sensor
// out of the filter. The observation gate is a quasi-static / interference
// detector on the RAW measurement magnitude; keying it on the bias-corrected
// magnitude let a runaway bias reject a clean measurement forever, so the
// attitude could never re-level. See docs/typst/mekf-errata.typ.
TEST(Mekf9Test, AccelGateUsesRawMagnitudeNotBiasCorrected) {
  Mekf9 mekf;
  Mekf9::Params p = DefaultParams();
  p.accel_gate_threshold = 0.5;
  p.init_accel_bias = Eigen::Vector3d(0.0, 0.0, 3.0);  // large, wrong accel bias
  mekf.Initialize(p);

  // Device is static: the raw accelerometer reads pure gravity (|a| = g), so the
  // gate must accept it regardless of the (diverged) bias estimate.
  const Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  ASSERT_TRUE(mekf.Update(Eigen::Vector3d::Zero(), GravityReading(q),
                          MagReading(q), 0.01));
  EXPECT_TRUE(mekf.LastUpdateUsedAccel());  // false under the |a - b_f| gate
}

TEST(Mekf9Test, MagGateUsesRawMagnitudeNotBiasCorrected) {
  Mekf9 mekf;
  Mekf9::Params p = DefaultParams();
  p.mag_gate_threshold = 0.2 * kMagRef.norm();
  p.init_mag_bias = kMagRef;  // large, wrong mag bias (~100% of the field)
  mekf.Initialize(p);

  const Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  ASSERT_TRUE(mekf.Update(Eigen::Vector3d::Zero(), GravityReading(q),
                          MagReading(q), 0.01));
  EXPECT_TRUE(mekf.LastUpdateUsedMag());  // false under the |m - b_m| gate
}
