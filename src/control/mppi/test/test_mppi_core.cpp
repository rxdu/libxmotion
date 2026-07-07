/*
 * test_mppi_core.cpp
 *
 * Unit tests for the MPPI machinery: weight computation, warm-start shift,
 * constraint clamping, determinism, and diagnostics.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <cmath>

#include "xmnav/mppi/critics.hpp"
#include "xmnav/mppi/models/diff_drive.hpp"
#include "xmnav/mppi/mppi.hpp"

using namespace xmotion;

TEST(MppiCoreTest, SoftmaxWeightsMatchHandComputation) {
  Eigen::VectorXd costs(3);
  costs << 10.0, 12.0, 14.0;
  Eigen::VectorXd w;
  mppi_detail::SoftmaxWeights(costs, /*lambda=*/2.0, w);

  // baseline rho = 10: exponents 0, -1, -2
  const double e0 = 1.0, e1 = std::exp(-1.0), e2 = std::exp(-2.0);
  const double eta = e0 + e1 + e2;
  EXPECT_NEAR(w(0), e0 / eta, 1e-12);
  EXPECT_NEAR(w(1), e1 / eta, 1e-12);
  EXPECT_NEAR(w(2), e2 / eta, 1e-12);
  EXPECT_NEAR(w.sum(), 1.0, 1e-12);
}

TEST(MppiCoreTest, SoftmaxSurvivesHugeCosts) {
  // without baseline subtraction these would all underflow to zero
  Eigen::VectorXd costs(2);
  costs << 1e8, 1e8 + 1.0;
  Eigen::VectorXd w;
  mppi_detail::SoftmaxWeights(costs, 1.0, w);
  EXPECT_TRUE(w.allFinite());
  EXPECT_GT(w(0), w(1));
  EXPECT_NEAR(w.sum(), 1.0, 1e-12);
}

TEST(MppiCoreTest, ShiftDropsExecutedStepAndPadsTail) {
  Eigen::Matrix<double, Eigen::Dynamic, 2> u(3, 2);
  u << 1, 10, 2, 20, 3, 30;
  mppi_detail::ShiftSequence(u);
  EXPECT_DOUBLE_EQ(u(0, 0), 2);
  EXPECT_DOUBLE_EQ(u(1, 0), 3);
  EXPECT_DOUBLE_EQ(u(2, 0), 3);  // tail holds the last value
  EXPECT_DOUBLE_EQ(u(2, 1), 30);
}

namespace {
using GoalMppi = Mppi<DiffDriveModel, Se2GoalCost>;

GoalMppi MakeController(std::uint64_t seed) {
  Se2GoalCost cost;
  cost.goal << 1.0, 0.0, 0.0;
  GoalMppi::Params p;
  p.num_samples = 256;
  p.horizon_steps = 20;
  p.dt = 0.05;
  p.lambda = 0.5;
  p.sigma << 0.3, 0.6;
  p.u_min << -0.5, -1.0;
  p.u_max << 0.5, 1.0;
  p.seed = seed;
  return GoalMppi(DiffDriveModel{}, cost, p);
}
}  // namespace

TEST(MppiCoreTest, DeterministicUnderFixedSeed) {
  auto a = MakeController(7);
  auto b = MakeController(7);
  const GoalMppi::State x0 = GoalMppi::State::Zero();
  for (int i = 0; i < 5; ++i) {
    a.Plan(x0);
    b.Plan(x0);
  }
  EXPECT_TRUE(a.Sequence().isApprox(b.Sequence()));
}

TEST(MppiCoreTest, CommandsRespectBoxConstraints) {
  auto c = MakeController(3);
  GoalMppi::State x = GoalMppi::State::Zero();
  for (int i = 0; i < 50; ++i) {
    const auto& seq = c.Plan(x);
    for (Eigen::Index t = 0; t < seq.rows(); ++t) {
      EXPECT_LE(seq(t, 0), 0.5 + 1e-12);
      EXPECT_GE(seq(t, 0), -0.5 - 1e-12);
      EXPECT_LE(seq(t, 1), 1.0 + 1e-12);
      EXPECT_GE(seq(t, 1), -1.0 - 1e-12);
    }
    x = DiffDriveModel{}.Step(x, c.Command(), 0.05);
  }
}

TEST(MppiCoreTest, DiagnosticsAreSane) {
  auto c = MakeController(11);
  c.Plan(GoalMppi::State::Zero());
  EXPECT_GT(c.LastEffectiveSampleSize(), 1.0);
  EXPECT_LE(c.LastEffectiveSampleSize(), 256.0);
  EXPECT_TRUE(std::isfinite(c.LastBestCost()));
}
