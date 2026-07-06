/*
 * @file sampler.hpp
 * @brief Sampling distributions for MPPI.
 *
 * The sampler is a deliberate seam (see docs/typst/mppi.typ): most post-2020
 * MPPI research (log-MPPI, colored noise, annealed covariance) lives entirely
 * in how the perturbation sequences are drawn. A sampler fills the
 * preallocated noise buffers with zero-mean perturbations scaled per channel.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MPPI_SAMPLER_HPP
#define XMNAV_MPPI_SAMPLER_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include <eigen3/Eigen/Dense>

namespace xmotion {

// i.i.d. Gaussian perturbations, N(0, diag(sigma^2)) per channel — the base
// distribution of the information-theoretic derivation (Williams 2018).
template <int ControlDim>
class GaussianSampler {
 public:
  using Control = Eigen::Matrix<double, ControlDim, 1>;
  using ControlSequence = Eigen::Matrix<double, Eigen::Dynamic, ControlDim>;

  explicit GaussianSampler(std::uint64_t seed) : generator_(seed) {}

  void SampleNoise(std::vector<ControlSequence> &noise, const Control &sigma) {
    for (auto &sequence : noise) {
      for (Eigen::Index t = 0; t < sequence.rows(); ++t) {
        for (int c = 0; c < ControlDim; ++c) {
          sequence(t, c) = sigma(c) * unit_normal_(generator_);
        }
      }
    }
  }

 private:
  std::mt19937_64 generator_;
  std::normal_distribution<double> unit_normal_{0.0, 1.0};
};

// Time-correlated (low-pass / Ornstein-Uhlenbeck style) perturbations:
// eps_t = beta * eps_{t-1} + sqrt(1 - beta^2) * w_t, preserving the marginal
// per-channel variance. Low-frequency samples explore smoothly and reduce
// chattering (Vlahov et al. 2024, arXiv:2404.03094).
template <int ControlDim>
class ColoredNoiseSampler {
 public:
  using Control = Eigen::Matrix<double, ControlDim, 1>;
  using ControlSequence = Eigen::Matrix<double, Eigen::Dynamic, ControlDim>;

  ColoredNoiseSampler(std::uint64_t seed, double beta)
      : generator_(seed), beta_(beta) {}

  void SampleNoise(std::vector<ControlSequence> &noise, const Control &sigma) {
    const double fresh = std::sqrt(1.0 - beta_ * beta_);
    for (auto &sequence : noise) {
      for (Eigen::Index t = 0; t < sequence.rows(); ++t) {
        for (int c = 0; c < ControlDim; ++c) {
          const double w = sigma(c) * unit_normal_(generator_);
          sequence(t, c) =
              (t == 0) ? w : beta_ * sequence(t - 1, c) + fresh * w;
        }
      }
    }
  }

 private:
  std::mt19937_64 generator_;
  double beta_;
  std::normal_distribution<double> unit_normal_{0.0, 1.0};
};

// Normal-log-normal mixture perturbations (log-MPPI, Mohamed et al. 2022,
// arXiv:2203.16599): eps = n * exp(z), n ~ N(0, sigma^2), z ~ N(-s^2/2, s^2)
// so E[exp(z)] = 1. Heavier tails at the same nominal scale produce more
// feasible rollouts in cluttered spaces.
template <int ControlDim>
class LogMppiSampler {
 public:
  using Control = Eigen::Matrix<double, ControlDim, 1>;
  using ControlSequence = Eigen::Matrix<double, Eigen::Dynamic, ControlDim>;

  LogMppiSampler(std::uint64_t seed, double lognormal_sigma)
      : generator_(seed), ln_sigma_(lognormal_sigma) {}

  void SampleNoise(std::vector<ControlSequence> &noise, const Control &sigma) {
    const double mu = -0.5 * ln_sigma_ * ln_sigma_;
    for (auto &sequence : noise) {
      for (Eigen::Index t = 0; t < sequence.rows(); ++t) {
        for (int c = 0; c < ControlDim; ++c) {
          const double n = sigma(c) * unit_normal_(generator_);
          const double z = mu + ln_sigma_ * unit_normal_(generator_);
          sequence(t, c) = n * std::exp(z);
        }
      }
    }
  }

 private:
  std::mt19937_64 generator_;
  double ln_sigma_;
  std::normal_distribution<double> unit_normal_{0.0, 1.0};
};

// Spline-knot perturbations: noise is sampled at a small number of knots and
// linearly interpolated across the horizon. Reduces the effective decision
// dimension by ~an order of magnitude and yields smooth controls by
// construction — the enabling technique of legged/whole-body sampling MPC
// (predictive sampling, whole-body MPPI; see the technical note). The
// weighted-average update stays inside the spline subspace automatically.
template <int ControlDim>
class SplineKnotSampler {
 public:
  using Control = Eigen::Matrix<double, ControlDim, 1>;
  using ControlSequence = Eigen::Matrix<double, Eigen::Dynamic, ControlDim>;

  SplineKnotSampler(std::uint64_t seed, int num_knots)
      : generator_(seed), num_knots_(num_knots < 2 ? 2 : num_knots) {}

  void SampleNoise(std::vector<ControlSequence> &noise, const Control &sigma) {
    for (auto &sequence : noise) {
      const Eigen::Index horizon = sequence.rows();
      knots_.resize(num_knots_, ControlDim);
      for (int k = 0; k < num_knots_; ++k) {
        for (int c = 0; c < ControlDim; ++c) {
          knots_(k, c) = sigma(c) * unit_normal_(generator_);
        }
      }
      const double span =
          static_cast<double>(horizon - 1) / (num_knots_ - 1);
      for (Eigen::Index t = 0; t < horizon; ++t) {
        const double pos = (span > 0.0) ? static_cast<double>(t) / span : 0.0;
        const int k0 = std::min(static_cast<int>(pos), num_knots_ - 2);
        const double frac = pos - k0;
        for (int c = 0; c < ControlDim; ++c) {
          sequence(t, c) =
              (1.0 - frac) * knots_(k0, c) + frac * knots_(k0 + 1, c);
        }
      }
    }
  }

 private:
  std::mt19937_64 generator_;
  int num_knots_;
  std::normal_distribution<double> unit_normal_{0.0, 1.0};
  Eigen::Matrix<double, Eigen::Dynamic, ControlDim> knots_;
};

}  // namespace xmotion

#endif  // XMNAV_MPPI_SAMPLER_HPP
