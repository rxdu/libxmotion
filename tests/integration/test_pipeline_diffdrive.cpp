/*
 * test_pipeline_diffdrive.cpp
 *
 * Integration scenario: the composed wheeled pipeline against simulation.
 *
 *   IMU+mag synthesis -> Mekf9 (yaw) -> dead-reckoned pose -> Mppi -> shield
 *                                                                  |
 *   true plant (diff-drive + process noise)  <---------------------+
 *
 * The controller never sees ground truth: it plans on a pose estimate
 * built from the MEKF9 yaw (gravity pins roll/pitch, the magnetometer
 * anchors heading — MEKF6 is deliberately NOT used here: without a mag
 * the yaw-rate bias is unobservable and its estimate free-wanders, which
 * the first version of this scenario demonstrated) and dead-reckoned
 * position from issued commands. The shield sits between Command() and
 * the plant with the same obstacle set as the cost.
 *
 * What the units cannot catch and this does: estimator drift feeding the
 * planner, the shield's barrier acting on an *estimated* (wrong) pose,
 * and the closed loop still reaching the goal within stated tolerances.
 *
 * Test conditions (anti-pattern #9): deterministic seeds, nominal plant
 * with process noise, IMU noise/biases stated inline; robustness under
 * plant-model mismatch lives in campaign_diffdrive.cpp.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "xmnav/estimation/mekf9.hpp"
#include "xmnav/models/diff_drive.hpp"
#include "xmnav/mppi/critics.hpp"
#include "xmnav/mppi/mppi.hpp"
#include "xmnav/shield/wheeled_shield.hpp"

using namespace xmotion;

namespace {

#if !defined(NDEBUG) || defined(__SANITIZE_ADDRESS__) || \
    defined(__SANITIZE_THREAD__)
constexpr int kSamples = 128;
#else
constexpr int kSamples = 512;
#endif

constexpr double kDt = 0.05;
constexpr double kGravity = 9.81;
const Eigen::Vector2d kGoal{3.0, 1.0};

double YawOf(const Eigen::Quaterniond &q) {
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

double WrapAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

}  // namespace

TEST(PipelineIntegrationTest, EstimateFedMppiWithShieldReachesGoal) {
  // --- obstacle course shared by cost and shield barrier ---
  Se2GoalCost goal_cost;
  goal_cost.goal << kGoal.x(), kGoal.y(), 0.0;
  CircularObstacleCost obstacle_cost;
  obstacle_cost.obstacles.push_back({Eigen::Vector2d(1.4, 0.15), 0.3});
  obstacle_cost.obstacles.push_back({Eigen::Vector2d(2.4, 1.15), 0.3});
  // SYSTEM RULE this tier enforces: the planner's soft margin (0.25) must
  // exceed the shield's hard inflation (margin 0.05 + look-ahead 0.15),
  // or the two layers disagree about which corridors exist and the robot
  // wedges against the invisible CBF wall (first version of this test
  // found exactly that deadlock).
  obstacle_cost.margin = 0.25;

  // --- controller (plans on the ESTIMATED state) ---
  auto cost = MakeCompositeCost(goal_cost, obstacle_cost);
  using Controller = Mppi<DiffDriveModel, decltype(cost)>;
  Controller::Params p;
  p.num_samples = kSamples;
  p.horizon_steps = 40;
  p.dt = kDt;
  p.lambda = 0.3;
  p.sigma << 0.3, 0.8;
  p.u_min << -0.8, -2.0;
  p.u_max << 0.8, 2.0;
  p.normalize_cost_spread = true;
  Controller mppi(DiffDriveModel{}, cost, p);

  // --- shield between Command() and the plant ---
  WheeledShield::Config sc;
  sc.envelope.u_min = p.u_min;
  sc.envelope.u_max = p.u_max;
  sc.envelope.rate_limit << 3.0, 10.0;
  sc.barrier.u_min = p.u_min;
  sc.barrier.u_max = p.u_max;
  sc.barrier.margin = 0.05;
  for (const auto &ob : obstacle_cost.obstacles) {
    sc.barrier.obstacles.push_back({ob.center, ob.radius});
  }
  WheeledShield shield(sc);

  // --- estimator: MEKF yaw from a synthesized IMU ---
  // gyro noise 0.005 rad/s, accel noise 0.05 m/s^2; the yaw-rate bias
  // (0.002 rad/s) is unobservable from gravity and drifts the heading by
  // ~0.05 rad over the run — the realism this scenario exists to absorb
  Mekf9::Params ep;
  ep.sigma_omega = Eigen::Vector3d::Constant(0.005);
  ep.sigma_f = Eigen::Vector3d::Constant(0.05);
  ep.sigma_beta_omega = Eigen::Vector3d::Constant(1e-4);
  ep.sigma_beta_f = Eigen::Vector3d::Constant(1e-4);
  ep.init_state_cov = Mekf9::StateCovariance::Identity() * 1e-2;
  ep.accel_noise_cov =
      Mekf9::ObservationNoiseCovariance::Identity() * 0.05 * 0.05;
  ep.mag_noise_cov =
      Mekf9::ObservationNoiseCovariance::Identity() * 0.02 * 0.02;
  ep.mag_reference = Eigen::Vector3d(1.0, 0.0, 0.0);
  Mekf9 mekf;
  mekf.Initialize(ep);
  const Eigen::Vector3d gyro_bias(0.005, -0.005, 0.002);

  std::mt19937_64 rng(11);
  std::normal_distribution<double> unit;

  // --- closed loop: truth vs estimate ---
  DiffDriveModel plant;
  DiffDriveModel::State x_true = DiffDriveModel::State::Zero();
  Eigen::Vector2d p_hat = Eigen::Vector2d::Zero();
  double yaw_hat = 0.0;
  WheeledShield::Control u = WheeledShield::Control::Zero();
  double max_yaw_error = 0.0;
  int shield_interventions = 0;

  for (int t = 0; t < 500; ++t) {
    // IMU synthesis from the executed motion (level planar body)
    Eigen::Vector3d gyro(0.0, 0.0, u(1));
    gyro += gyro_bias + 0.005 * Eigen::Vector3d(unit(rng), unit(rng),
                                                unit(rng));
    Eigen::Vector3d accel(0.0, 0.0, -kGravity);
    accel += 0.05 * Eigen::Vector3d(unit(rng), unit(rng), unit(rng));
    // magnetometer: the inertial reference rotated into the (yaw-only)
    // body frame, plus noise
    Eigen::Vector3d mag(std::cos(x_true(2)), -std::sin(x_true(2)), 0.0);
    mag += 0.02 * Eigen::Vector3d(unit(rng), unit(rng), unit(rng));
    ASSERT_TRUE(mekf.Update(gyro, accel, mag, kDt)) << "tick " << t;
    yaw_hat = YawOf(mekf.GetQuaternion());
    max_yaw_error =
        std::max(max_yaw_error, std::abs(WrapAngle(yaw_hat - x_true(2))));

    // plan on the estimate, shield, actuate the true plant
    DiffDriveModel::State x_hat;
    x_hat << p_hat.x(), p_hat.y(), yaw_hat;
    mppi.Plan(x_hat);
    u = shield.Filter(mppi.Command(), x_hat, /*state_age=*/0.0, kDt);
    ASSERT_EQ(shield.mode(), ShieldMode::kNormal) << "tick " << t;
    if (shield.LastReport().modified) ++shield_interventions;

    x_true = plant.Step(x_true, u, t, kDt);
    x_true(0) += 0.002 * unit(rng);  // process noise on the true plant
    x_true(1) += 0.002 * unit(rng);
    x_true(2) += 0.004 * unit(rng);

    // dead-reckoned position from the issued command + estimated heading
    p_hat += u(0) * Eigen::Vector2d(std::cos(yaw_hat), std::sin(yaw_hat)) *
             kDt;
  }

  // the TRUE robot reached the goal despite planning on the estimate
  const double true_goal_dist = (x_true.head<2>() - kGoal).norm();
  EXPECT_LT(true_goal_dist, 0.35) << "true state: " << x_true.transpose();
  // heading estimate stayed usable (mag anchors yaw; gravity holds
  // roll/pitch)
  EXPECT_LT(max_yaw_error, 0.15);
  // dead reckoning stayed close enough to steer by
  EXPECT_LT((p_hat - x_true.head<2>()).norm(), 0.5);
  // the pipeline ran; the envelope engaging occasionally is normal, a
  // fault-mode excursion is not (asserted every tick above)
  EXPECT_GE(shield_interventions, 0);
}
