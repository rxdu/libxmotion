/*
 * test_mppi_control.cpp
 *
 * Behavioral validation of the MPPI controller: point stabilization and
 * obstacle avoidance on the differential-drive model, and a closed-loop
 * cross-check against the discrete LQR solution on the double integrator —
 * the analytic oracle for this implementation.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <cmath>

#include "xmnav/mppi/critics.hpp"
#include "xmnav/models/diff_drive.hpp"
#include "xmnav/models/double_integrator.hpp"
#include "xmnav/mppi/mppi.hpp"

using namespace xmotion;

namespace {
// Reduced scale under sanitizer/Debug builds: exercise the code paths, keep
// the numerical convergence assertions in Release where they are validated.
#if !defined(NDEBUG) || defined(__SANITIZE_ADDRESS__) || \
    defined(__SANITIZE_THREAD__)
constexpr bool kReducedScale = true;
#else
constexpr bool kReducedScale = false;
#endif
}  // namespace


TEST(MppiControlTest, DiffDriveReachesGoalPose) {
  Se2GoalCost cost;
  cost.goal << 2.0, 1.0, M_PI / 2.0;

  using Controller = Mppi<DiffDriveModel, Se2GoalCost>;
  Controller::Params p;
  p.num_samples = kReducedScale ? 128 : 1024;
  p.horizon_steps = 40;
  p.dt = 0.05;
  p.lambda = 0.3;
  p.sigma << 0.3, 0.8;
  p.u_min << -0.8, -2.0;
  p.u_max << 0.8, 2.0;
  Controller mppi(DiffDriveModel{}, cost, p);

  DiffDriveModel model;
  Controller::State x = Controller::State::Zero();
  for (int i = 0; i < (kReducedScale ? 30 : 400); ++i) {  // 20 s in Release
    mppi.Plan(x);
    x = model.Step(x, mppi.Command(), 0, p.dt);
  }

  if (!kReducedScale) {
    EXPECT_NEAR(x(0), 2.0, 0.15);
    EXPECT_NEAR(x(1), 1.0, 0.15);
    EXPECT_LT(std::abs(std::remainder(x(2) - M_PI / 2.0, 2.0 * M_PI)), 0.3);
  } else {
    EXPECT_TRUE(x.allFinite());
  }
}

TEST(MppiControlTest, DiffDriveAvoidsObstacleEnRoute) {
  Se2GoalCost goal_cost;
  goal_cost.goal << 3.0, 0.0, 0.0;

  CircularObstacleCost obstacle_cost;
  obstacle_cost.obstacles.push_back({Eigen::Vector2d(1.5, 0.0), 0.4});

  auto cost = MakeCompositeCost(goal_cost, obstacle_cost);
  using Controller = Mppi<DiffDriveModel, decltype(cost)>;
  Controller::Params p;
  p.num_samples = kReducedScale ? 128 : 1024;
  p.horizon_steps = 50;
  p.dt = 0.05;
  p.lambda = 0.3;
  p.sigma << 0.3, 0.8;
  p.u_min << -0.8, -2.0;
  p.u_max << 0.8, 2.0;
  Controller mppi(DiffDriveModel{}, cost, p);

  DiffDriveModel model;
  Controller::State x = Controller::State::Zero();
  double min_clearance = 1e9;
  for (int i = 0; i < (kReducedScale ? 30 : 500); ++i) {
    mppi.Plan(x);
    x = model.Step(x, mppi.Command(), 0, p.dt);
    min_clearance = std::min(
        min_clearance, (x.head<2>() - Eigen::Vector2d(1.5, 0.0)).norm() - 0.4);
  }

  EXPECT_GT(min_clearance, 0.0) << "executed path entered the obstacle";
  if (!kReducedScale) {
    EXPECT_NEAR(x(0), 3.0, 0.2);
    EXPECT_NEAR(x(1), 0.0, 0.2);
  }
}

namespace {

// discrete-time LQR by Riccati iteration; returns the gain and the
// converged cost-to-go matrix P
Eigen::RowVector2d LqrGain(const Eigen::Matrix2d& A, const Eigen::Vector2d& B,
                           const Eigen::Matrix2d& Q, double R,
                           Eigen::Matrix2d* P_out) {
  Eigen::Matrix2d P = Q;
  for (int i = 0; i < 1000; ++i) {
    const double S = R + B.dot(P * B);
    const Eigen::RowVector2d K = (B.transpose() * P * A) / S;
    P = Q + A.transpose() * P * (A - B * K);
  }
  const double S = R + B.dot(P * B);
  if (P_out != nullptr) *P_out = P;
  return (B.transpose() * P * A) / S;
}

}  // namespace

TEST(MppiControlTest, MatchesLqrOnDoubleIntegrator) {
  const double dt = 0.05;
  const Eigen::Matrix2d Q = Eigen::Vector2d(10.0, 1.0).asDiagonal();
  const double R = 0.1;

  // --- LQR closed loop (the oracle) ---
  Eigen::Matrix2d A;
  A << 1, dt, 0, 1;
  const Eigen::Vector2d B(0, dt);
  Eigen::Matrix2d P_lqr;
  const Eigen::RowVector2d K = LqrGain(A, B, Q, R, &P_lqr);

  const Eigen::Vector2d x0(1.0, 0.0);
  double lqr_cost = 0.0;
  {
    Eigen::Vector2d x = x0;
    for (int i = 0; i < 120; ++i) {
      const double u = -(K * x)(0);
      lqr_cost += x.dot(Q * x) + R * u * u;
      x = A * x + B * u;
    }
  }

  // --- MPPI closed loop with the same stage cost ---
  QuadraticStateCost<2, 1> state_cost;
  state_cost.Q = Q;
  state_cost.Q_terminal = P_lqr;  // truncated horizon == infinite horizon
  QuadraticControlCost<2, 1> control_cost;
  control_cost.R << R;
  auto cost = MakeCompositeCost(state_cost, control_cost);

  using Controller = Mppi<DoubleIntegratorModel, decltype(cost)>;
  Controller::Params p;
  p.num_samples = kReducedScale ? 128 : 2048;
  p.horizon_steps = 40;
  p.dt = dt;
  // operating point from the parameter sweep: lambda must be scaled to the
  // per-horizon cost spread (see the technical note on temperature)
  p.lambda = 1.0;
  p.control_cost_decoupling = 1.0;  // pure cost minimization for comparison
  p.sigma << 1.5;
  Controller mppi(DoubleIntegratorModel{}, cost, p);

  DoubleIntegratorModel model;
  double mppi_cost = 0.0;
  {
    Controller::State x = x0;
    for (int i = 0; i < (kReducedScale ? 20 : 120); ++i) {
      mppi.Plan(x);
      const double u = mppi.Command()(0);
      mppi_cost += x.dot(Q * x) + R * u * u;
      x = model.Step(x, mppi.Command(), 0, dt);
    }
  }

  if (!kReducedScale) {
    // sampling-based control approaches but cannot beat the analytic optimum
    EXPECT_GT(mppi_cost, 0.95 * lqr_cost);
    EXPECT_LT(mppi_cost, 1.25 * lqr_cost)
        << "MPPI closed-loop cost " << mppi_cost << " vs LQR " << lqr_cost;
  } else {
    EXPECT_TRUE(std::isfinite(mppi_cost));
  }
}
