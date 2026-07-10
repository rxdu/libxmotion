/*
 * campaign_bicycle_mismatch.cpp
 *
 * Robustness campaign (ctest label: campaign — nightly): the classic
 * kinematic-planner vs dynamic-plant benchmark (Kong et al., IV 2015).
 * MPPI plans with the KINEMATIC bicycle; the true plant is the
 * single-track linear-tire model (Rajamani) that slips, with per-seed
 * tire-stiffness variation — the canonical model-mismatch axis.
 *
 * Interface between the two: the planner commands [v, delta]; the plant
 * runs a longitudinal speed loop ax = kv (v_cmd - vx) and takes delta
 * directly. Acceptance is the pass rate across seeds.
 *
 * Mismatch (stated test conditions):
 *   - planner model: kinematic bicycle, wheelbase 2.8 m (matches lf+lr)
 *   - plant: dynamic bicycle, Cf/Cr independently scaled U(0.7, 1.3)
 *   - speed loop kv = 1.5 1/s; process noise (2 mm, 2 mm, 1 mrad)
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <random>

#include "xmnav/models/ackermann.hpp"
#include "xmnav/models/dynamic_bicycle.hpp"
#include "xmnav/mppi/critics.hpp"
#include "xmnav/mppi/mppi.hpp"

using namespace xmotion;

namespace {

#if !defined(NDEBUG) || defined(__SANITIZE_ADDRESS__) || \
    defined(__SANITIZE_THREAD__)
constexpr int kSamples = 128;
constexpr int kRuns = 4;
#else
constexpr int kSamples = 512;
constexpr int kRuns = 25;
#endif

constexpr double kDt = 0.05;
const Eigen::Vector2d kGoal{40.0, 6.0};  // a lane-change-and-advance task

struct RunResult {
  bool reached = false;
  bool spun = false;
  double final_dist = 0.0;
};

RunResult RunOnce(std::uint64_t seed) {
  Se2GoalCost goal_cost;
  goal_cost.goal << kGoal.x(), kGoal.y(), 0.0;
  goal_cost.position_weight = 1.0;

  AckermannModel planner_model;
  planner_model.wheelbase = 2.8;  // matches the plant's lf + lr

  using Controller = Mppi<AckermannModel, Se2GoalCost>;
  Controller::Params p;
  p.num_samples = kSamples;
  p.horizon_steps = 30;
  p.dt = kDt;
  p.lambda = 0.3;
  p.sigma << 1.5, 0.05;
  p.u_min << 0.0, -0.4;
  p.u_max << 15.0, 0.4;
  p.normalize_cost_spread = true;
  p.seed = seed;
  Controller mppi(planner_model, goal_cost, p);

  std::mt19937_64 rng(seed ^ 0xda3e39cb94b95bdbULL);
  std::normal_distribution<double> unit;
  std::uniform_real_distribution<double> tire(0.7, 1.3);

  DynamicBicycleModel plant;
  plant.cornering_front *= tire(rng);
  plant.cornering_rear *= tire(rng);

  DynamicBicycleModel::State x = DynamicBicycleModel::State::Zero();
  x(3) = 10.0;  // entering at speed
  RunResult result;
  for (int t = 0; t < 200; ++t) {  // 10 s budget
    AckermannModel::State x_planner;
    x_planner << x(0), x(1), x(2);
    mppi.Plan(x_planner);
    const auto cmd = mppi.Command();  // [v_cmd, delta]
    DynamicBicycleModel::Control u;
    u << 1.5 * (cmd(0) - x(3)), cmd(1);  // speed loop + direct steering
    x = plant.Step(x, u, t, kDt);
    x(0) += 0.002 * unit(rng);
    x(1) += 0.002 * unit(rng);
    x(2) += 0.001 * unit(rng);
    if (std::abs(x(4)) > 3.0) {  // lateral velocity blow-up = spin
      result.spun = true;
      break;
    }
    if ((x.head<2>() - kGoal).norm() < 1.5) {
      result.reached = true;
      break;
    }
  }
  result.final_dist = (x.head<2>() - kGoal).norm();
  return result;
}

}  // namespace

TEST(BicycleMismatchCampaignTest, KinematicPlannerHandlesTireSlipPlant) {
  int reached = 0, spins = 0;
  for (int i = 0; i < kRuns; ++i) {
    const auto r = RunOnce(2000 + static_cast<std::uint64_t>(i));
    reached += r.reached ? 1 : 0;
    spins += r.spun ? 1 : 0;
    if (!r.reached) {
      std::printf("  seed %d: final_dist=%.2f spun=%d\n", 2000 + i,
                  r.final_dist, r.spun ? 1 : 0);
    }
  }
  const double pass_rate = static_cast<double>(reached) / kRuns;
  std::printf("campaign: %d/%d reached (%.0f%%), %d spins\n", reached,
              kRuns, 100.0 * pass_rate, spins);
  EXPECT_GE(pass_rate, 0.9);
  EXPECT_EQ(spins, 0);
}
