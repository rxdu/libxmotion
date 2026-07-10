/*
 * campaign_diffdrive.cpp
 *
 * Robustness campaign (ctest label: campaign — nightly, not PR CI): the
 * wheeled MPPI + shield loop under plant-model mismatch, Monte Carlo over
 * seeds. The controller plans with the nominal kinematic model; the true
 * plant executes with an actuator gain deficit and a first-order lag —
 * unmodeled dynamics the nominal-plant tests never exercise. Acceptance
 * is a PASS RATE across the seed population, not a single trajectory.
 *
 * Mismatch (stated test conditions):
 *   - actuator gain 0.90 on v, 0.95 on omega
 *   - first-order actuator lag tau = 80 ms on both channels
 *   - process noise sigma = (2 mm, 2 mm, 4 mrad) per step
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <random>

#include "xmnav/models/diff_drive.hpp"
#include "xmnav/mppi/critics.hpp"
#include "xmnav/mppi/mppi.hpp"
#include "xmnav/shield/wheeled_shield.hpp"

using namespace xmotion;

namespace {

#if !defined(NDEBUG) || defined(__SANITIZE_ADDRESS__) || \
    defined(__SANITIZE_THREAD__)
constexpr int kSamples = 128;
constexpr int kRuns = 6;
#else
constexpr int kSamples = 512;
constexpr int kRuns = 40;
#endif

constexpr double kDt = 0.05;
const Eigen::Vector2d kGoal{3.0, 1.0};

struct RunResult {
  bool reached = false;
  bool shield_fault = false;
  double final_dist = 0.0;
  int ticks_to_goal = 0;
};

RunResult RunOnce(std::uint64_t seed) {
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
  p.seed = seed;
  Controller mppi(DiffDriveModel{}, cost, p);

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

  std::mt19937_64 rng(seed ^ 0x9e3779b97f4a7c15ULL);
  std::normal_distribution<double> unit;

  // true plant with unmodeled actuator dynamics
  const Eigen::Vector2d gain(0.90, 0.95);
  const double lag_alpha = kDt / (0.08 + kDt);
  Eigen::Vector2d u_actuated = Eigen::Vector2d::Zero();

  DiffDriveModel plant;
  DiffDriveModel::State x = DiffDriveModel::State::Zero();
  RunResult result;
  for (int t = 0; t < 600; ++t) {
    mppi.Plan(x);  // full state feedback; mismatch is the subject here
    const auto u = shield.Filter(mppi.Command(), x, 0.0, kDt);
    if (shield.mode() != ShieldMode::kNormal) {
      result.shield_fault = true;
      break;
    }
    u_actuated += lag_alpha * (gain.cwiseProduct(u) - u_actuated);
    x = plant.Step(x, u_actuated, t, kDt);
    x(0) += 0.002 * unit(rng);
    x(1) += 0.002 * unit(rng);
    x(2) += 0.004 * unit(rng);
    if ((x.head<2>() - kGoal).norm() < 0.3) {
      result.reached = true;
      result.ticks_to_goal = t;
      break;
    }
  }
  result.final_dist = (x.head<2>() - kGoal).norm();
  return result;
}

}  // namespace

TEST(DiffDriveCampaignTest, GoalReachedUnderActuatorMismatch) {
  int reached = 0, faults = 0;
  double worst_dist = 0.0;
  for (int i = 0; i < kRuns; ++i) {
    const auto r = RunOnce(1000 + static_cast<std::uint64_t>(i));
    reached += r.reached ? 1 : 0;
    faults += r.shield_fault ? 1 : 0;
    worst_dist = std::max(worst_dist, r.final_dist);
    if (!r.reached) {
      std::printf("  seed %d: final_dist=%.3f fault=%d\n", 1000 + i,
                  r.final_dist, r.shield_fault ? 1 : 0);
    }
  }
  const double pass_rate = static_cast<double>(reached) / kRuns;
  std::printf("campaign: %d/%d reached (%.0f%%), worst final dist %.3f\n",
              reached, kRuns, 100.0 * pass_rate, worst_dist);
  EXPECT_GE(pass_rate, 0.9) << "goal-reach rate under actuator mismatch";
  EXPECT_EQ(faults, 0) << "shield fault-mode excursions";
}
