/*
 * test_mppi_cuda.cpp
 *
 * Validation of the CUDA rollout backend against the CPU reference: the
 * device evaluates the same model/cost cores in float32, so costs must
 * match the CPU double results to float accumulation error (not bitwise),
 * and the controller must behave identically in closed loop. All tests
 * skip when no CUDA device is present (CI has none).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <cmath>

#include "xmnav/mppi/cuda/cuda_rollout_backend.hpp"
#include "xmnav/mppi/critics.hpp"
#include "xmnav/mppi/models/diff_drive.hpp"
#include "xmnav/mppi/mppi.hpp"

using namespace xmotion;

namespace {

using Cost = CudaWheeledRolloutBackend::Cost;
using Params = CudaWheeledRolloutBackend::Params;
using ControlSequence = CudaWheeledRolloutBackend::ControlSequence;

Cost MakeScenarioCost() {
  Se2GoalCost goal;
  goal.goal << 3.5, 1.0, 0.0;
  CircularObstacleCost obstacles;
  obstacles.obstacles.push_back({Eigen::Vector2d(1.5, 0.2), 0.4});
  obstacles.obstacles.push_back({Eigen::Vector2d(2.6, 1.1), 0.3});
  return MakeCompositeCost(goal, obstacles);
}

Params MakeScenarioParams() {
  Params p;
  p.num_samples = 1024;
  p.horizon_steps = 50;
  p.dt = 0.05;
  p.lambda = 0.3;
  p.sigma << 0.3, 0.8;
  p.u_min << -0.8, -2.0;
  p.u_max << 0.8, 2.0;
  p.normalize_cost_spread = true;
  return p;
}

#define SKIP_WITHOUT_DEVICE()                              \
  if (!CudaDeviceAvailable()) {                            \
    GTEST_SKIP() << "no CUDA device available on this host"; \
  }

}  // namespace

TEST(MppiCudaTest, CostsMatchCpuWithinFloatTolerance) {
  SKIP_WITHOUT_DEVICE();
  const Cost cost = MakeScenarioCost();
  const Params p = MakeScenarioParams();
  const DiffDriveModel model;

  // one shared noise realization for both backends
  GaussianSampler<2> sampler(p.seed);
  std::vector<ControlSequence> noise(
      static_cast<std::size_t>(p.num_samples),
      ControlSequence::Zero(p.horizon_steps, 2));
  sampler.SampleNoise(noise, p.sigma);

  ControlSequence u = ControlSequence::Zero(p.horizon_steps, 2);
  u.col(0).setConstant(0.4);  // non-trivial nominal so the IS term is active
  const CudaWheeledRolloutBackend::State x0 =
      CudaWheeledRolloutBackend::State::Zero();
  const CudaWheeledRolloutBackend::Control sigma_inv_sq =
      p.sigma.cwiseProduct(p.sigma).cwiseInverse();
  const double gamma = p.lambda * (1.0 - p.control_cost_decoupling);

  Eigen::VectorXd cpu_costs(p.num_samples);
  Eigen::VectorXd gpu_costs(p.num_samples);
  CpuRolloutBackend<DiffDriveModel, Cost> cpu;
  cpu.Evaluate(model, cost, p, gamma, x0, u, noise, sigma_inv_sq, cpu_costs);
  CudaWheeledRolloutBackend gpu;
  gpu.Evaluate(model, cost, p, gamma, x0, u, noise, sigma_inv_sq, gpu_costs);

  ASSERT_TRUE(gpu_costs.allFinite());
  double worst_rel = 0.0;
  for (int k = 0; k < p.num_samples; ++k) {
    const double rel = std::abs(gpu_costs(k) - cpu_costs(k)) /
                       std::max(1.0, std::abs(cpu_costs(k)));
    worst_rel = std::max(worst_rel, rel);
  }
  // float32 rollouts over a 50-step horizon: ~1e-5 typical, 1e-3 budget
  EXPECT_LT(worst_rel, 1e-3) << "float/double divergence too large";
  EXPECT_NEAR(gpu_costs.minCoeff(), cpu_costs.minCoeff(),
              1e-3 * std::max(1.0, std::abs(cpu_costs.minCoeff())));
}

TEST(MppiCudaTest, ProgramFollowsCriticMutations) {
  SKIP_WITHOUT_DEVICE();
  Cost cost = MakeScenarioCost();
  const Params p = MakeScenarioParams();
  const DiffDriveModel model;

  GaussianSampler<2> sampler(p.seed);
  std::vector<ControlSequence> noise(
      static_cast<std::size_t>(p.num_samples),
      ControlSequence::Zero(p.horizon_steps, 2));
  sampler.SampleNoise(noise, p.sigma);
  const ControlSequence u = ControlSequence::Zero(p.horizon_steps, 2);
  const CudaWheeledRolloutBackend::State x0 =
      CudaWheeledRolloutBackend::State::Zero();
  const CudaWheeledRolloutBackend::Control sigma_inv_sq =
      p.sigma.cwiseProduct(p.sigma).cwiseInverse();

  CudaWheeledRolloutBackend gpu;
  Eigen::VectorXd costs_a(p.num_samples), costs_b(p.num_samples);
  gpu.Evaluate(model, cost, p, 0.0, x0, u, noise, sigma_inv_sq, costs_a);
  // a different goal must produce different device-side costs (the program
  // is rebuilt from the live cost object on every call)
  Cost moved_goal = MakeScenarioCost();
  Se2GoalCost far_goal;
  far_goal.goal << -5.0, -5.0, 0.0;
  CircularObstacleCost same_obstacles;
  same_obstacles.obstacles.push_back({Eigen::Vector2d(1.5, 0.2), 0.4});
  same_obstacles.obstacles.push_back({Eigen::Vector2d(2.6, 1.1), 0.3});
  moved_goal = MakeCompositeCost(far_goal, same_obstacles);
  gpu.Evaluate(model, moved_goal, p, 0.0, x0, u, noise, sigma_inv_sq,
               costs_b);
  EXPECT_GT((costs_a - costs_b).cwiseAbs().maxCoeff(), 1.0);
}

TEST(MppiCudaTest, ClosedLoopReachesGoalThroughObstacles) {
  SKIP_WITHOUT_DEVICE();
  const Cost cost = MakeScenarioCost();
  const Params p = MakeScenarioParams();
  using Controller =
      Mppi<DiffDriveModel, Cost, GaussianSampler<2>, CudaWheeledRolloutBackend>;
  Controller mppi(DiffDriveModel{}, cost, p, GaussianSampler<2>(p.seed),
                  CudaWheeledRolloutBackend{});

  DiffDriveModel plant;
  DiffDriveModel::State x = DiffDriveModel::State::Zero();
  for (int t = 0; t < 400; ++t) {
    mppi.Plan(x);
    x = plant.Step(x, mppi.Command(), 0, p.dt);
  }
  const double dist =
      std::hypot(x(0) - 3.5, x(1) - 1.0);
  EXPECT_LT(dist, 0.2) << "final state: " << x.transpose();
  EXPECT_GT(mppi.LastEffectiveSampleSize(), 1.0);
  EXPECT_TRUE(std::isfinite(mppi.LastBestCost()));
}
