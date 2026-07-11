/*
 * test_allocation.cpp
 *
 * Zero-heap-allocation contracts of the steady-state hot paths (the
 * "adopt now" item of docs/research/ihmc-open-robotics-software.md §6):
 * Mppi::Plan(), WheeledShield::Filter(), Mekf6/Mekf9::Update(), and
 * PidController::Update(). Each test constructs and warms up outside the
 * probe window (construction and first-call buffer sizing may allocate),
 * then asserts that many steady-state iterations allocate nothing.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <cmath>

#include "alloc_counter.hpp"
#include "xmnav/estimation/mekf6.hpp"
#include "xmnav/estimation/mekf9.hpp"
#include "xmnav/models/diff_drive.hpp"
#include "xmnav/mppi/critics.hpp"
#include "xmnav/mppi/mppi.hpp"
#include "xmnav/pid/pid_controller.hpp"
#include "xmnav/shield/wheeled_shield.hpp"

using namespace xmotion;
using alloc_test::AllocationProbe;
using alloc_test::AllocCountingActive;

namespace {

constexpr int kSteadyStateIters = 1000;
constexpr double kGravity = 9.81;
const Eigen::Vector3d kMagRef(0.5, 0.0, -0.6);

#define XMNAV_REQUIRE_ALLOC_COUNTING()                                  \
  if (!AllocCountingActive())                                           \
  GTEST_SKIP() << "malloc interposition inactive (sanitizer or non-glibc)"

// accelerometer reading for a body at rest with attitude q (body-to-inertial)
Eigen::Vector3d GravityReading(const Eigen::Quaterniond &q_true) {
  return q_true.conjugate() * Eigen::Vector3d(0, 0, -kGravity);
}

Eigen::Vector3d MagReading(const Eigen::Quaterniond &q_true) {
  return q_true.conjugate() * kMagRef;
}

Mekf6::Params Mekf6Params() {
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

Mekf9::Params Mekf9Params() {
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

}  // namespace

TEST(AllocationTest, PidUpdateAllocatesNothing) {
  XMNAV_REQUIRE_ALLOC_COUNTING();

  PidController::Config cfg;
  cfg.kp = 2.0;
  cfg.ki = 0.5;
  cfg.kd = 0.1;
  cfg.d_filter_tau = 0.05;
  cfg.u_min = -1.0;
  cfg.u_max = 1.0;
  // non-empty name: telemetry handles are acquired at construction, which
  // may allocate — Update() must not
  cfg.name = "alloc_probe";
  PidController pid(cfg);

  const double dt = 0.01;
  for (int i = 0; i < 10; ++i) pid.Update(1.0, 0.1 * i, dt);

  double u = 0.0;
  AllocationProbe probe;
  for (int i = 0; i < kSteadyStateIters; ++i) {
    u = pid.Update(1.0, std::sin(0.01 * i), dt);
  }
  EXPECT_EQ(probe.Count(), 0) << "PidController::Update allocated";
  EXPECT_TRUE(std::isfinite(u));
}

TEST(AllocationTest, Mekf6UpdateAllocatesNothing) {
  XMNAV_REQUIRE_ALLOC_COUNTING();

  const Eigen::Quaterniond q_true =
      Eigen::AngleAxisd(10.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(-7.0 * M_PI / 180.0, Eigen::Vector3d::UnitY());
  Mekf6 mekf;
  mekf.Initialize(Mekf6Params());

  const double dt = 0.01;
  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  const Eigen::Vector3d accel = GravityReading(q_true);
  for (int i = 0; i < 10; ++i) ASSERT_TRUE(mekf.Update(gyro, accel, dt));

  AllocationProbe probe;
  bool ok = true;
  for (int i = 0; i < kSteadyStateIters; ++i) {
    ok = ok && mekf.Update(gyro, accel, dt);
  }
  EXPECT_EQ(probe.Count(), 0) << "Mekf6::Update allocated";
  EXPECT_TRUE(ok);
}

TEST(AllocationTest, Mekf9UpdateAllocatesNothing) {
  XMNAV_REQUIRE_ALLOC_COUNTING();

  const Eigen::Quaterniond q_true =
      Eigen::AngleAxisd(30.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(10.0 * M_PI / 180.0, Eigen::Vector3d::UnitX());
  Mekf9 mekf;
  mekf.Initialize(Mekf9Params());

  const double dt = 0.01;
  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  const Eigen::Vector3d accel = GravityReading(q_true);
  const Eigen::Vector3d mag = MagReading(q_true);
  for (int i = 0; i < 10; ++i) ASSERT_TRUE(mekf.Update(gyro, accel, mag, dt));

  AllocationProbe probe;
  bool ok = true;
  for (int i = 0; i < kSteadyStateIters; ++i) {
    ok = ok && mekf.Update(gyro, accel, mag, dt);
  }
  EXPECT_EQ(probe.Count(), 0) << "Mekf9::Update allocated";
  EXPECT_TRUE(ok);
}

TEST(AllocationTest, MppiPlanAllocatesNothing) {
  XMNAV_REQUIRE_ALLOC_COUNTING();

  Se2GoalCost cost;
  cost.goal << 2.0, 1.0, M_PI / 2.0;

  using Controller = Mppi<DiffDriveModel, Se2GoalCost>;
  Controller::Params p;
  p.num_samples = 256;
  p.horizon_steps = 40;
  p.dt = 0.05;
  p.lambda = 0.3;
  p.sigma << 0.3, 0.8;
  p.u_min << -0.8, -2.0;
  p.u_max << 0.8, 2.0;
  // SavitzkyGolay5 copies the sequence every call — a documented
  // conditional path, out of the steady-state zero-alloc contract
  p.smooth_output = false;
  Controller mppi(DiffDriveModel{}, cost, p);

  DiffDriveModel model;
  Controller::State x = Controller::State::Zero();
  // first Plan() sizes the weighted-update buffer; warm up past it
  for (int i = 0; i < 3; ++i) {
    mppi.Plan(x);
    x = model.Step(x, mppi.Command(), 0, p.dt);
  }

  AllocationProbe probe;
  for (int i = 0; i < 100; ++i) {
    mppi.Plan(x);
    x = model.Step(x, mppi.Command(), 0, p.dt);
  }
  EXPECT_EQ(probe.Count(), 0) << "Mppi::Plan allocated";
  EXPECT_TRUE(x.allFinite());
}

TEST(AllocationTest, WheeledShieldFilterAllocatesNothing) {
  XMNAV_REQUIRE_ALLOC_COUNTING();

  // No obstacles: the barrier path builds per-call std::vectors and the
  // QP solver allocates per solve (known limitations, tracked in the
  // backlog) — this pins the envelope + ladder passthrough path.
  WheeledShield::Config cfg;
  cfg.envelope.u_min << -0.8, -2.0;
  cfg.envelope.u_max << 0.8, 2.0;
  cfg.envelope.rate_limit << 2.0, 8.0;  // [m/s^2, rad/s^2]
  cfg.ladder.hold_timeout = 0.1;
  cfg.ladder.stop_ramp_time = 0.2;
  cfg.state_staleness_max = 0.1;
  cfg.enable_barrier = false;
  WheeledShield shield(cfg);

  const double dt = 0.02;
  const WheeledShield::State origin = WheeledShield::State::Zero();
  WheeledShield::Control u = WheeledShield::Control::Zero();
  for (int i = 0; i < 50; ++i) {
    u = shield.Filter(WheeledShield::Control(0.5, 0.0), origin, 0.0, dt);
  }
  ASSERT_EQ(shield.mode(), ShieldMode::kNormal);

  AllocationProbe probe;
  for (int i = 0; i < kSteadyStateIters; ++i) {
    const WheeledShield::Control cmd(0.5 + 0.2 * std::sin(0.01 * i),
                                     0.1 * std::cos(0.01 * i));
    u = shield.Filter(cmd, origin, 0.0, dt);
  }
  EXPECT_EQ(probe.Count(), 0) << "WheeledShield::Filter allocated";
  EXPECT_EQ(shield.mode(), ShieldMode::kNormal);
  EXPECT_TRUE(u.allFinite());
}
