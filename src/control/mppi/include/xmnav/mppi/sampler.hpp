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

}  // namespace xmotion

#endif  // XMNAV_MPPI_SAMPLER_HPP
