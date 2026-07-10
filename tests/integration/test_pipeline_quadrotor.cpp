/*
 * test_pipeline_quadrotor.cpp
 *
 * Integration scenario: quadrotor waypoint capture and hold with MPPI
 * planning directly on the rigid-body dynamics (Mellinger/Kumar model),
 * gravity-compensated warm start, process noise on velocity and body
 * rates. Exercises the controller on a genuinely 3D attitude state
 * (quaternion + body rates) — the planar scenarios cannot.
 *
 * Test conditions: nominal plant (the controller's own model) with
 * additive process noise; deterministic seeds.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "xmnav/models/quadrotor.hpp"
#include "xmnav/mppi/mppi.hpp"

using namespace xmotion;

namespace {

#if !defined(NDEBUG) || defined(__SANITIZE_ADDRESS__) || \
    defined(__SANITIZE_THREAD__)
constexpr int kSamples = 128;
#else
constexpr int kSamples = 512;
#endif

constexpr double kDt = 0.02;
const Eigen::Vector3d kWaypoint{2.0, 1.0, 1.5};

struct WaypointCost {
  using State = QuadrotorModel::State;
  using Control = QuadrotorModel::Control;
  double hover_thrust = 0.5 * 9.81;
  double StageCost(const State &x, const Control &u, int /*t*/) const {
    const Eigen::Vector3d pe = QuadrotorModel::Position(x) - kWaypoint;
    const Eigen::Vector3d v = QuadrotorModel::Velocity(x);
    const Eigen::Vector3d w = QuadrotorModel::BodyRates(x);
    // tilt via R(2,2) of the unit quaternion (1 when level)
    const double body_z_z =
        1.0 - 2.0 * (x(7) * x(7) + x(8) * x(8));
    const double du = u(0) - hover_thrust;
    return 4.0 * pe.squaredNorm() + 1.0 * v.squaredNorm() +
           20.0 * (1.0 - body_z_z) + 0.05 * w.squaredNorm() +
           0.1 * du * du + 2.0 * u.tail<3>().squaredNorm();
  }
  double TerminalCost(const State &x) const {
    return 10.0 * StageCost(x, Control::Zero(), 0);
  }
};

}  // namespace

TEST(PipelineIntegrationTest, QuadrotorReachesAndHoldsWaypoint) {
  QuadrotorModel model;
  using Controller = Mppi<QuadrotorModel, WaypointCost>;
  Controller::Params p;
  p.num_samples = kSamples;
  p.horizon_steps = 50;  // 1 s lookahead
  p.dt = kDt;
  p.lambda = 0.2;
  p.sigma << 1.0, 0.02, 0.02, 0.01;
  p.u_min << 0.0, -0.05, -0.05, -0.02;
  p.u_max << 2.0 * model.HoverThrust(), 0.05, 0.05, 0.02;
  p.normalize_cost_spread = true;
  WaypointCost cost;
  cost.hover_thrust = model.HoverThrust();
  Controller mppi(model, cost, p);
  // gravity-compensating warm start (the standard force-space seed)
  Controller::Control hover = Controller::Control::Zero();
  hover(0) = model.HoverThrust();
  mppi.SeedSequence(hover);

  std::mt19937_64 rng(5);
  std::normal_distribution<double> unit;

  auto x = QuadrotorModel::MakeState(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
  double max_tilt = 0.0;
  for (int t = 0; t < 400; ++t) {  // 8 s
    mppi.Plan(x);
    x = model.Step(x, mppi.Command(), t, kDt);
    for (int i = 3; i < 6; ++i) x(i) += 0.005 * unit(rng);   // v noise
    for (int i = 10; i < 13; ++i) x(i) += 0.002 * unit(rng); // w noise
    const double body_z_z = 1.0 - 2.0 * (x(7) * x(7) + x(8) * x(8));
    max_tilt = std::max(max_tilt, 1.0 - body_z_z);
    ASSERT_TRUE(x.allFinite()) << "tick " << t;
  }

  EXPECT_LT((QuadrotorModel::Position(x) - kWaypoint).norm(), 0.3)
      << "final position: " << QuadrotorModel::Position(x).transpose();
  EXPECT_LT(QuadrotorModel::Velocity(x).norm(), 0.5);
  EXPECT_LT(max_tilt, 0.3) << "excessive tilt during the maneuver";
}
