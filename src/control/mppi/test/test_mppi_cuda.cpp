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
#include "xmnav/models/diff_drive.hpp"
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

// --- M4c: SRB quadruped program ---

namespace {

constexpr double kSrbDt = 0.02;
constexpr int kSrbHorizon = 20;
constexpr double kSrbHeight = 0.28;

const std::array<Eigen::Vector3d, 4> kFootOffsets = {
    Eigen::Vector3d(0.19, 0.11, 0.0), Eigen::Vector3d(0.19, -0.11, 0.0),
    Eigen::Vector3d(-0.19, 0.11, 0.0), Eigen::Vector3d(-0.19, -0.11, 0.0)};

SrbQuadrupedModel::FootPositions FeetUnderBody(const Eigen::Vector3d &p) {
  SrbQuadrupedModel::FootPositions feet;
  for (std::size_t i = 0; i < 4; ++i) {
    feet[i] = Eigen::Vector3d(p(0), p(1), 0.0) + kFootOffsets[i];
  }
  return feet;
}

SrbQuadrupedModel::ContactSchedule AllStance() {
  return SrbQuadrupedModel::ContactSchedule(kSrbHorizon,
                                            {true, true, true, true});
}

// diagonal-pair trot phases over the horizon, starting at cycle_step
SrbQuadrupedModel::ContactSchedule TrotSchedule(int cycle_step,
                                                int phase_steps) {
  SrbQuadrupedModel::ContactSchedule s(kSrbHorizon);
  for (int t = 0; t < kSrbHorizon; ++t) {
    const bool diag_a = (((cycle_step + t) / phase_steps) % 2) == 0;
    s[static_cast<std::size_t>(t)] = {diag_a, !diag_a, !diag_a, diag_a};
  }
  return s;
}

using SrbCost = CudaSrbRolloutBackend::Cost;
using SrbParams = CudaSrbRolloutBackend::Params;
using SrbSeq = CudaSrbRolloutBackend::ControlSequence;

SrbCost MakeSrbCost(const SrbQuadrupedModel::ContactSchedule &schedule) {
  SrbTrackingCost tracking;
  tracking.height_ref = kSrbHeight;
  FrictionConeCost cone;
  cone.schedule = schedule;
  QuadraticControlCost<13, 12> reg;
  reg.R = Eigen::Matrix<double, 12, 12>::Identity() * 1e-4;
  return MakeCompositeCost(tracking, cone, reg);
}

SrbParams MakeSrbParams() {
  SrbParams p;
  p.num_samples = 1024;
  p.horizon_steps = kSrbHorizon;
  p.dt = kSrbDt;
  p.lambda = 0.1;
  p.control_cost_decoupling = 1.0;
  p.normalize_cost_spread = true;
  for (int i = 0; i < 4; ++i) {
    p.sigma.segment<3>(3 * i) << 8.0, 8.0, 15.0;
    p.u_min.segment<3>(3 * i) << -60.0, -60.0, 0.0;
    p.u_max.segment<3>(3 * i) << 60.0, 60.0, 160.0;
  }
  return p;
}

SrbQuadrupedModel::State SrbStandingState() {
  return SrbQuadrupedModel::MakeState(
      Eigen::Vector3d(0, 0, kSrbHeight), Eigen::Vector3d::Zero(),
      Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
}

// gravity-compensating stance seed: mg/4 vertical on each foot
SrbQuadrupedModel::Control GravitySeed(const SrbQuadrupedModel &model) {
  SrbQuadrupedModel::Control u = SrbQuadrupedModel::Control::Zero();
  const double fz = model.params().mass * model.params().gravity / 4.0;
  for (int i = 0; i < 4; ++i) u(3 * i + 2) = fz;
  return u;
}

void ExpectSrbCostsMatch(const SrbQuadrupedModel &model, const SrbCost &cost,
                         const SrbParams &p,
                         const SrbQuadrupedModel::State &x0) {
  SplineKnotSampler<12> sampler(p.seed, 5);
  std::vector<SrbSeq> noise(static_cast<std::size_t>(p.num_samples),
                            SrbSeq::Zero(p.horizon_steps, 12));
  sampler.SampleNoise(noise, p.sigma);
  SrbSeq u = GravitySeed(model).transpose().replicate(p.horizon_steps, 1);
  const CudaSrbRolloutBackend::Control sigma_inv_sq =
      p.sigma.cwiseProduct(p.sigma).cwiseInverse();
  const double gamma = p.lambda * (1.0 - p.control_cost_decoupling);

  Eigen::VectorXd cpu_costs(p.num_samples), gpu_costs(p.num_samples);
  CpuRolloutBackend<SrbQuadrupedModel, SrbCost> cpu;
  cpu.Evaluate(model, cost, p, gamma, x0, u, noise, sigma_inv_sq, cpu_costs);
  CudaSrbRolloutBackend gpu;
  gpu.Evaluate(model, cost, p, gamma, x0, u, noise, sigma_inv_sq, gpu_costs);

  ASSERT_TRUE(gpu_costs.allFinite());
  double worst_rel = 0.0;
  for (int k = 0; k < p.num_samples; ++k) {
    const double rel = std::abs(gpu_costs(k) - cpu_costs(k)) /
                       std::max(1.0, std::abs(cpu_costs(k)));
    worst_rel = std::max(worst_rel, rel);
  }
  // 20-step float rollouts with quaternion renormalization: 1e-3 budget
  EXPECT_LT(worst_rel, 1e-3) << "float/double divergence too large";
}

}  // namespace

TEST(MppiCudaSrbTest, StandingCostsMatchCpu) {
  SKIP_WITHOUT_DEVICE();
  SrbQuadrupedModel model;
  const auto x0 = SrbStandingState();
  model.SetContext(FeetUnderBody(SrbQuadrupedModel::Position(x0)),
                   AllStance());
  ExpectSrbCostsMatch(model, MakeSrbCost(AllStance()), MakeSrbParams(), x0);
}

TEST(MppiCudaSrbTest, TrotCostsMatchCpu) {
  SKIP_WITHOUT_DEVICE();
  SrbQuadrupedModel model;
  const auto x0 = SrbStandingState();
  const auto schedule = TrotSchedule(3, 5);
  // per-step foot plan: constant here — the per-step upload path is what
  // this exercises, together with swing-force masking in the kernel
  std::vector<SrbQuadrupedModel::FootPositions> plan(
      kSrbHorizon, FeetUnderBody(SrbQuadrupedModel::Position(x0)));
  model.SetContext(plan, schedule);
  ExpectSrbCostsMatch(model, MakeSrbCost(schedule), MakeSrbParams(), x0);
}

TEST(MppiCudaSrbTest, ScheduleMismatchIsRejected) {
  SKIP_WITHOUT_DEVICE();
  SrbQuadrupedModel model;
  const auto x0 = SrbStandingState();
  model.SetContext(FeetUnderBody(SrbQuadrupedModel::Position(x0)),
                   AllStance());
  const SrbCost cost = MakeSrbCost(TrotSchedule(0, 5));  // disagrees
  const SrbParams p = MakeSrbParams();
  SplineKnotSampler<12> sampler(p.seed, 5);
  std::vector<SrbSeq> noise(static_cast<std::size_t>(p.num_samples),
                            SrbSeq::Zero(p.horizon_steps, 12));
  sampler.SampleNoise(noise, p.sigma);
  const SrbSeq u = SrbSeq::Zero(p.horizon_steps, 12);
  const CudaSrbRolloutBackend::Control sigma_inv_sq =
      p.sigma.cwiseProduct(p.sigma).cwiseInverse();
  Eigen::VectorXd costs(p.num_samples);
  CudaSrbRolloutBackend gpu;
  EXPECT_THROW(gpu.Evaluate(model, cost, p, 0.0, x0, u, noise, sigma_inv_sq,
                            costs),
               std::invalid_argument);
}

TEST(MppiCudaSrbTest, StandingBalanceClosedLoopOnGpu) {
  SKIP_WITHOUT_DEVICE();
  using Controller = Mppi<SrbQuadrupedModel, SrbCost, SplineKnotSampler<12>,
                          CudaSrbRolloutBackend>;
  const SrbParams p = MakeSrbParams();
  Controller mppi(SrbQuadrupedModel{}, MakeSrbCost(AllStance()), p,
                  SplineKnotSampler<12>(p.seed, 5), CudaSrbRolloutBackend{});
  mppi.SeedSequence(GravitySeed(mppi.model()));

  SrbQuadrupedModel plant;
  auto x = SrbStandingState();
  for (int i = 0; i < 150; ++i) {  // 3 s
    const auto feet = FeetUnderBody(SrbQuadrupedModel::Position(x));
    mppi.model().SetContext(feet, AllStance());
    plant.SetContext(feet, AllStance());
    mppi.Plan(x);
    x = plant.Step(x, mppi.Command(), 0, kSrbDt);
  }
  EXPECT_NEAR(SrbQuadrupedModel::Position(x)(2), kSrbHeight, 0.03);
  const Eigen::Vector3d body_z =
      SrbQuadrupedModel::Orientation(x) * Eigen::Vector3d::UnitZ();
  EXPECT_GT(body_z(2), 0.99) << "trunk tilted";
}

// --- M4b: on-device sampling ---

TEST(MppiCudaSamplingTest, DeterministicPerSeed) {
  SKIP_WITHOUT_DEVICE();
  using Controller = Mppi<DiffDriveModel, Cost, GaussianSampler<2>,
                          CudaWheeledSamplingBackend>;
  const Params p = MakeScenarioParams();
  Controller a(DiffDriveModel{}, MakeScenarioCost(), p, GaussianSampler<2>(1),
               CudaWheeledSamplingBackend(99));
  Controller b(DiffDriveModel{}, MakeScenarioCost(), p, GaussianSampler<2>(1),
               CudaWheeledSamplingBackend(99));
  const DiffDriveModel::State x0 = DiffDriveModel::State::Zero();
  for (int i = 0; i < 5; ++i) {
    a.Plan(x0);
    b.Plan(x0);
  }
  EXPECT_TRUE((a.Sequence().array() == b.Sequence().array()).all());
  // a different seed must give a different plan
  Controller c(DiffDriveModel{}, MakeScenarioCost(), p, GaussianSampler<2>(1),
               CudaWheeledSamplingBackend(100));
  for (int i = 0; i < 5; ++i) c.Plan(x0);
  EXPECT_FALSE((a.Sequence().array() == c.Sequence().array()).all());
}

TEST(MppiCudaSamplingTest, DeviceNoiseHasRequestedMoments) {
  SKIP_WITHOUT_DEVICE();
  const Cost cost = MakeScenarioCost();
  Params p = MakeScenarioParams();
  p.num_samples = 2048;
  const DiffDriveModel model;
  const CudaWheeledRolloutBackend::State x0 =
      CudaWheeledRolloutBackend::State::Zero();
  const ControlSequence u = ControlSequence::Zero(p.horizon_steps, 2);
  const CudaWheeledRolloutBackend::Control sigma_inv_sq =
      p.sigma.cwiseProduct(p.sigma).cwiseInverse();
  Eigen::VectorXd costs(p.num_samples);
  CudaWheeledSamplingBackend gpu(7);
  gpu.Evaluate(model, cost, p, 0.0, x0, u, {}, sigma_inv_sq, costs);

  // sample moments over all K*T draws per channel
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  Eigen::Vector2d sq = Eigen::Vector2d::Zero();
  ControlSequence eps;
  const double n = static_cast<double>(p.num_samples) * p.horizon_steps;
  for (int k = 0; k < p.num_samples; ++k) {
    gpu.DownloadNoiseSample(k, eps);
    mean += eps.colwise().sum().transpose() / n;
    sq += eps.array().square().colwise().sum().transpose().matrix() / n;
  }
  for (int j = 0; j < 2; ++j) {
    const double std_dev = std::sqrt(sq(j) - mean(j) * mean(j));
    EXPECT_NEAR(mean(j), 0.0, 0.02 * p.sigma(j)) << "channel " << j;
    EXPECT_NEAR(std_dev, p.sigma(j), 0.03 * p.sigma(j)) << "channel " << j;
  }
}

TEST(MppiCudaSamplingTest, WeightedUpdateMatchesHostRecomputation) {
  SKIP_WITHOUT_DEVICE();
  const Cost cost = MakeScenarioCost();
  const Params p = MakeScenarioParams();
  const DiffDriveModel model;
  const CudaWheeledRolloutBackend::State x0 =
      CudaWheeledRolloutBackend::State::Zero();
  const ControlSequence u = ControlSequence::Zero(p.horizon_steps, 2);
  const CudaWheeledRolloutBackend::Control sigma_inv_sq =
      p.sigma.cwiseProduct(p.sigma).cwiseInverse();
  Eigen::VectorXd costs(p.num_samples);
  CudaWheeledSamplingBackend gpu(7);
  gpu.Evaluate(model, cost, p, 0.0, x0, u, {}, sigma_inv_sq, costs);

  Eigen::VectorXd weights;
  mppi_detail::SoftmaxWeights(costs, p.lambda, weights);
  ControlSequence device_delta;
  gpu.ApplyWeightedUpdate(weights, device_delta);

  ControlSequence host_delta = ControlSequence::Zero(p.horizon_steps, 2);
  ControlSequence eps;
  for (int k = 0; k < p.num_samples; ++k) {
    gpu.DownloadNoiseSample(k, eps);
    host_delta += weights(k) * eps;
  }
  EXPECT_LT((device_delta - host_delta).cwiseAbs().maxCoeff(), 1e-4);
}

TEST(MppiCudaSamplingTest, ClosedLoopReachesGoalThroughObstacles) {
  SKIP_WITHOUT_DEVICE();
  using Controller = Mppi<DiffDriveModel, Cost, GaussianSampler<2>,
                          CudaWheeledSamplingBackend>;
  const Params p = MakeScenarioParams();
  Controller mppi(DiffDriveModel{}, MakeScenarioCost(), p,
                  GaussianSampler<2>(p.seed), CudaWheeledSamplingBackend(42));
  mppi.EnableIntrospection(8, 8);  // exercises DownloadNoiseSample path

  DiffDriveModel plant;
  DiffDriveModel::State x = DiffDriveModel::State::Zero();
  for (int t = 0; t < 400; ++t) {
    mppi.Plan(x);
    x = plant.Step(x, mppi.Command(), 0, p.dt);
  }
  EXPECT_LT(std::hypot(x(0) - 3.5, x(1) - 1.0), 0.2)
      << "final state: " << x.transpose();
  EXPECT_FALSE(mppi.LastSnapshot().candidates.empty());
  EXPECT_TRUE(mppi.LastSnapshot().nominal_states.allFinite());
}
