/*
 * test_attitude_init.cpp
 *
 * TRIAD / leveling / declination utilities: exact recovery without
 * noise, graceful degradation with it, degenerate-geometry rejection,
 * and the bootstrap actually helping Mekf9 (near-zero error from the
 * first tick instead of a convergence transient).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "xmnav/estimation/attitude_init.hpp"
#include "xmnav/estimation/mekf9.hpp"

using namespace xmotion;

namespace {

constexpr double kGravity = 9.81;
const Eigen::Vector3d kMagRef = MagReferenceEnu(0.1, 0.9);  // ~ Europe-ish

double AngularError(const Eigen::Quaterniond &a, const Eigen::Quaterniond &b) {
  return 2.0 * std::acos(std::min(1.0, std::abs(a.dot(b))));
}

Eigen::Quaterniond RandomAttitude(std::mt19937_64 &rng) {
  std::normal_distribution<double> unit;
  Eigen::Quaterniond q(unit(rng), unit(rng), unit(rng), unit(rng));
  q.normalize();
  return q;
}

}  // namespace

TEST(AttitudeInitTest, TriadRecoversRandomAttitudesExactly) {
  std::mt19937_64 rng(3);
  for (int i = 0; i < 200; ++i) {
    const Eigen::Quaterniond q_true = RandomAttitude(rng);
    const Eigen::Vector3d accel =
        q_true.conjugate() * Eigen::Vector3d(0, 0, -kGravity);
    const Eigen::Vector3d mag = q_true.conjugate() * kMagRef;
    Eigen::Quaterniond q_est;
    ASSERT_TRUE(TriadAttitude(accel, mag, kMagRef, &q_est)) << i;
    EXPECT_LT(AngularError(q_est, q_true), 1e-6) << i;
  }
}

TEST(AttitudeInitTest, TriadDegradesGracefullyWithNoise) {
  std::mt19937_64 rng(7);
  std::normal_distribution<double> unit;
  double worst = 0.0;
  for (int i = 0; i < 100; ++i) {
    const Eigen::Quaterniond q_true = RandomAttitude(rng);
    Eigen::Vector3d accel =
        q_true.conjugate() * Eigen::Vector3d(0, 0, -kGravity);
    Eigen::Vector3d mag = q_true.conjugate() * kMagRef;
    accel += 0.05 * Eigen::Vector3d(unit(rng), unit(rng), unit(rng));
    mag += 0.02 * Eigen::Vector3d(unit(rng), unit(rng), unit(rng));
    Eigen::Quaterniond q_est;
    ASSERT_TRUE(TriadAttitude(accel, mag, kMagRef, &q_est));
    worst = std::max(worst, AngularError(q_est, q_true));
  }
  // 0.05/9.81 and 0.02/|m| direction noise -> ~0.5-2 deg typical; the
  // worst of 100 draws lands on the weakly conditioned (near-vertical
  // field) geometry, hence the generous bound
  EXPECT_LT(worst, 0.15);
}

TEST(AttitudeInitTest, TriadRejectsDegenerateGeometry) {
  Eigen::Quaterniond q;
  // magnetic field parallel to gravity: heading undetermined
  EXPECT_FALSE(TriadAttitude({0, 0, -kGravity}, {0, 0, -1}, {0, 0, -1}, &q));
  // garbage inputs
  EXPECT_FALSE(TriadAttitude({0, 0, 0}, {1, 0, 0}, kMagRef, &q));
  EXPECT_FALSE(
      TriadAttitude({std::nan(""), 0, -9.81}, {1, 0, 0}, kMagRef, &q));
}

TEST(AttitudeInitTest, LevelFromAccelRecoversRollPitchWithZeroYaw) {
  std::mt19937_64 rng(11);
  std::uniform_real_distribution<double> ang(-1.0, 1.0);
  for (int i = 0; i < 100; ++i) {
    const double roll = ang(rng), pitch = 0.5 * ang(rng);
    const Eigen::Quaterniond q_true(
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
    const Eigen::Vector3d accel =
        q_true.conjugate() * Eigen::Vector3d(0, 0, -kGravity);
    Eigen::Quaterniond q_est;
    ASSERT_TRUE(LevelFromAccel(accel, &q_est));
    EXPECT_LT(AngularError(q_est, q_true), 1e-6) << i;
  }
}

TEST(AttitudeInitTest, MagReferenceEnuConventions) {
  // zero declination/inclination: due true north (ENU +y), horizontal
  EXPECT_TRUE(MagReferenceEnu(0.0, 0.0).isApprox(Eigen::Vector3d(0, 1, 0)));
  // positive declination rotates toward east (+x); positive dip points
  // DOWN, which is -z in the z-up frame
  const auto m = MagReferenceEnu(0.1, 0.9, 50.0);
  EXPECT_GT(m(0), 0.0);
  EXPECT_LT(m(2), 0.0);
  EXPECT_NEAR(m.norm(), 50.0, 1e-12);
}

// the point of the bootstrap: with TRIAD init the Mekf9 error is small
// from the FIRST tick; identity init starts ~90 deg off and needs its
// convergence transient
TEST(AttitudeInitTest, TriadBootstrapEliminatesMekf9Transient) {
  std::mt19937_64 rng(13);
  const Eigen::Quaterniond q_true =
      Eigen::Quaterniond(Eigen::AngleAxisd(1.6, Eigen::Vector3d(1, 1, 0.5)
                                                    .normalized()));
  const Eigen::Vector3d accel =
      q_true.conjugate() * Eigen::Vector3d(0, 0, -kGravity);
  const Eigen::Vector3d mag = q_true.conjugate() * kMagRef;

  Mekf9::Params ep;
  ep.sigma_omega = Eigen::Vector3d::Constant(0.005);
  ep.sigma_f = Eigen::Vector3d::Constant(0.05);
  ep.sigma_beta_omega = Eigen::Vector3d::Constant(1e-4);
  ep.sigma_beta_f = Eigen::Vector3d::Constant(1e-4);
  ep.sigma_beta_m = Eigen::Vector3d::Constant(1e-4);
  ep.accel_noise_cov = Mekf9::ObservationNoiseCovariance::Identity() * 0.05;
  ep.mag_noise_cov = Mekf9::ObservationNoiseCovariance::Identity() * 0.02;
  ep.mag_reference = kMagRef;

  Eigen::Quaterniond q0;
  ASSERT_TRUE(TriadAttitude(accel, mag, kMagRef, &q0));
  ep.init_quaternion = q0;
  Mekf9 boot;
  boot.Initialize(ep);
  // one static update; the estimate must already be at the truth
  std::normal_distribution<double> unit;
  const Eigen::Vector3d gyro =
      0.005 * Eigen::Vector3d(unit(rng), unit(rng), unit(rng));
  ASSERT_TRUE(boot.Update(gyro, accel, mag, 0.005));
  EXPECT_LT(AngularError(boot.GetQuaternion(), q_true), 0.02);

  Mekf9::Params identity_params = ep;
  identity_params.init_quaternion = Eigen::Quaterniond::Identity();
  Mekf9 cold;
  cold.Initialize(identity_params);
  ASSERT_TRUE(cold.Update(gyro, accel, mag, 0.005));
  EXPECT_GT(AngularError(cold.GetQuaternion(), q_true), 0.5);
}
