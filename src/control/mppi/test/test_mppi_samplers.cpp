/*
 * test_mppi_samplers.cpp
 *
 * Sampler variants, output smoothing, cost normalization, and the
 * single-core performance envelope.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>

#include "xmnav/mppi/critics.hpp"
#include "xmnav/mppi/models/diff_drive.hpp"
#include "xmnav/mppi/models/double_integrator.hpp"
#include "xmnav/mppi/mppi.hpp"

using namespace xmotion;

namespace {
template <typename Sampler>
std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> Draw(Sampler& sampler,
                                                           int k, int t) {
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> noise(
      static_cast<std::size_t>(k),
      Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(t));
  Eigen::Matrix<double, 1, 1> sigma;
  sigma << 1.0;
  sampler.SampleNoise(noise, sigma);
  return noise;
}

double LagOneAutocorrelation(
    const std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>>& noise) {
  double num = 0.0, den = 0.0;
  for (const auto& seq : noise) {
    for (Eigen::Index t = 1; t < seq.rows(); ++t) num += seq(t) * seq(t - 1);
    den += seq.squaredNorm();
  }
  return num / den;
}
}  // namespace

TEST(MppiSamplerTest, ColoredNoiseIsTimeCorrelatedGaussianIsNot) {
  GaussianSampler<1> white(1);
  ColoredNoiseSampler<1> colored(1, /*beta=*/0.8);

  auto w = Draw(white, 200, 50);
  auto c = Draw(colored, 200, 50);

  EXPECT_LT(std::abs(LagOneAutocorrelation(w)), 0.05);
  EXPECT_GT(LagOneAutocorrelation(c), 0.6);

  // marginal variance is preserved by the sqrt(1 - beta^2) scaling
  double var = 0.0;
  int n = 0;
  for (const auto& seq : c) {
    var += seq.squaredNorm();
    n += static_cast<int>(seq.rows());
  }
  EXPECT_NEAR(var / n, 1.0, 0.1);
}

TEST(MppiSamplerTest, LogMppiHasHeavierTailsAtSameScale) {
  GaussianSampler<1> gaussian(2);
  LogMppiSampler<1> nln(2, /*lognormal_sigma=*/0.7);

  auto g = Draw(gaussian, 400, 50);
  auto l = Draw(nln, 400, 50);

  auto kurtosis = [](const auto& noise) {
    double m2 = 0.0, m4 = 0.0;
    int n = 0;
    for (const auto& seq : noise) {
      for (Eigen::Index t = 0; t < seq.rows(); ++t) {
        m2 += seq(t) * seq(t);
        m4 += seq(t) * seq(t) * seq(t) * seq(t);
        ++n;
      }
    }
    m2 /= n;
    m4 /= n;
    return m4 / (m2 * m2);
  };

  EXPECT_NEAR(kurtosis(g), 3.0, 0.3);   // Gaussian kurtosis
  EXPECT_GT(kurtosis(l), 4.0);          // NLN is leptokurtic
}

TEST(MppiSamplerTest, SplineKnotNoiseIsPiecewiseLinear) {
  const int knots = 5, horizon = 41;
  SplineKnotSampler<1> spline(3, knots);
  auto noise = Draw(spline, 10, horizon);

  // between knots, second differences vanish
  const double span = double(horizon - 1) / (knots - 1);  // = 10
  for (const auto& seq : noise) {
    for (Eigen::Index t = 1; t + 1 < seq.rows(); ++t) {
      const bool at_knot = std::abs(std::remainder(t, span)) < 1e-9;
      if (!at_knot) {
        EXPECT_NEAR(seq(t + 1) - 2 * seq(t) + seq(t - 1), 0.0, 1e-9);
      }
    }
  }
}

TEST(MppiSamplerTest, SavitzkyGolayPreservesLinearAndSmoothsJagged) {
  // exact on polynomials up to the fit order
  Eigen::Matrix<double, Eigen::Dynamic, 1> linear(9);
  for (int i = 0; i < 9; ++i) linear(i) = 2.0 * i + 1.0;
  auto filtered = linear;
  mppi_detail::SavitzkyGolay5(filtered);
  EXPECT_TRUE(filtered.isApprox(linear, 1e-12));

  // damps an alternating (Nyquist) component
  Eigen::Matrix<double, Eigen::Dynamic, 1> jagged(9);
  for (int i = 0; i < 9; ++i) jagged(i) = (i % 2 == 0) ? 1.0 : -1.0;
  auto smoothed = jagged;
  mppi_detail::SavitzkyGolay5(smoothed);
  for (int i = 2; i < 7; ++i) {
    EXPECT_LT(std::abs(smoothed(i)), std::abs(jagged(i)));
  }
}

TEST(MppiSamplerTest, CostNormalizationRestoresEffectiveSampleSize) {
  // the double-integrator scenario where raw lambda=1.0 collapses to ESS~1
  const double dt = 0.05;
  QuadraticStateCost<2, 1> sc;
  sc.Q = Eigen::Vector2d(10.0, 1.0).asDiagonal();
  sc.Q_terminal = sc.Q;
  QuadraticControlCost<2, 1> cc;
  cc.R << 0.1;
  auto cost = MakeCompositeCost(sc, cc);
  using C = Mppi<DoubleIntegratorModel, decltype(cost)>;

  auto run = [&](bool normalize) {
    C::Params p;
    p.num_samples = 1024;
    p.horizon_steps = 40;
    p.dt = dt;
    p.lambda = normalize ? 0.2 : 1.0;  // normalized lambda is scale-free
    p.control_cost_decoupling = 1.0;
    p.sigma << 1.5;
    p.normalize_cost_spread = normalize;
    p.seed = 9;
    C mppi(DoubleIntegratorModel{}, cost, p);
    DoubleIntegratorModel m;
    C::State x(1.0, 0.0);
    double min_ess = 1e18;
    for (int i = 0; i < 60; ++i) {
      mppi.Plan(x);
      x = m.Step(x, mppi.Command(), dt);
      min_ess = std::min(min_ess, mppi.LastEffectiveSampleSize());
    }
    return min_ess;
  };

  EXPECT_LT(run(false), 3.0);    // raw: weight collapse
  EXPECT_GT(run(true), 20.0);    // normalized: healthy sample utilization
}

TEST(MppiSamplerTest, SplineSamplerYieldsSmootherCommandsOnDiffDrive) {
  Se2GoalCost goal;
  goal.goal << 2.0, 0.5, 0.0;

  // within-horizon roughness of the planned sequence: what a downstream
  // interpolator / low-level controller actually consumes. (Replan-to-replan
  // jitter of the executed command is a different quantity — coherent
  // per-sample noise makes samples more distinct, not less.)
  auto roughness = [&](auto&& mppi) {
    DiffDriveModel m;
    typename std::decay_t<decltype(mppi)>::State x =
        std::decay_t<decltype(mppi)>::State::Zero();
    double sum = 0.0;
    for (int i = 0; i < 200; ++i) {
      const auto& seq = mppi.Plan(x);
      for (Eigen::Index t = 0; t + 1 < seq.rows(); ++t) {
        sum += (seq.row(t + 1) - seq.row(t)).cwiseAbs().sum();
      }
      x = m.Step(x, mppi.Command(), 0.05);
    }
    return sum;
  };

  using Gauss = Mppi<DiffDriveModel, Se2GoalCost>;
  using Spline = Mppi<DiffDriveModel, Se2GoalCost, SplineKnotSampler<2>>;
  Gauss::Params p;
  p.num_samples = 512;
  p.horizon_steps = 40;
  p.dt = 0.05;
  p.lambda = 0.3;
  p.sigma << 0.3, 0.8;
  p.u_min << -0.8, -2.0;
  p.u_max << 0.8, 2.0;
  p.seed = 21;

  Gauss gauss(DiffDriveModel{}, goal, p);
  Spline spline(DiffDriveModel{}, goal, p, SplineKnotSampler<2>(21, 8));

  EXPECT_LT(roughness(spline), 0.7 * roughness(gauss));
}

TEST(MppiSamplerTest, SingleCorePerformanceEnvelope) {
#if !defined(NDEBUG) || defined(__SANITIZE_ADDRESS__) || \
    defined(__SANITIZE_THREAD__)
  GTEST_SKIP() << "timing bounds are only meaningful in optimized "
                  "non-sanitized builds";
#endif
  // production envelope: 1024 samples x 56 steps, diff-drive + goal critic.
  // The bound is deliberately loose for CI machines; the measured time is
  // printed for the record (target hardware budget: <10 ms).
  Se2GoalCost goal;
  goal.goal << 2.0, 1.0, 0.0;
  using C = Mppi<DiffDriveModel, Se2GoalCost>;
  C::Params p;
  p.num_samples = 1024;
  p.horizon_steps = 56;
  p.dt = 0.05;
  p.sigma << 0.3, 0.8;
  C mppi(DiffDriveModel{}, goal, p);

  C::State x = C::State::Zero();
  mppi.Plan(x);  // warm-up

  const auto start = std::chrono::steady_clock::now();
  const int iterations = 20;
  for (int i = 0; i < iterations; ++i) {
    mppi.Plan(x);
  }
  const auto elapsed = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - start)
                           .count() /
                       iterations;
  std::printf("[perf] 1024x56 diff-drive plan: %.2f ms/iter\n", elapsed);
  EXPECT_LT(elapsed, 50.0);
}
