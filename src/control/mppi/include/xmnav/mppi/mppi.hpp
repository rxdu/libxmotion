/*
 * @file mppi.hpp
 * @brief Model Predictive Path Integral controller.
 *
 * Implements the information-theoretic MPPI algorithm of Williams et al.,
 * "Information-Theoretic Model Predictive Control: Theory and Applications
 * to Autonomous Driving", IEEE T-RO 34(6), 2018 (arXiv:1707.02342). The
 * derivation, symbol conventions, and implementation decisions are recorded
 * in docs/typst/mppi.typ.
 *
 * Design: the controller is templated on three seams —
 *   Model:   static kStateDim/kControlDim; State Step(State, Control, dt)
 *   Cost:    double StageCost(state, control, t); double TerminalCost(state)
 *   Sampler: void SampleNoise(noise_buffers, sigma)
 * The hot path (Plan) performs no heap allocation: all rollout buffers are
 * sized at construction. Sampling is deterministic under a fixed seed.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MPPI_MPPI_HPP
#define XMNAV_MPPI_MPPI_HPP

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "xmnav/mppi/sampler.hpp"

namespace xmotion {
namespace mppi_detail {

// w_k = exp(-(S_k - rho)/lambda) / eta, rho = min S (baseline subtraction —
// mandatory numerically, see the note). Returns eta before normalization so
// callers can detect degeneracy.
inline double SoftmaxWeights(const Eigen::VectorXd &costs, double lambda,
                             Eigen::VectorXd &weights) {
  const double rho = costs.minCoeff();
  weights = (-(costs.array() - rho) / lambda).exp();
  const double eta = weights.sum();
  weights /= eta;
  return eta;
}

// Receding-horizon warm start: drop the executed first step, shift the rest
// forward, hold the last control on the padded tail.
template <typename Sequence>
inline void ShiftSequence(Sequence &u) {
  const Eigen::Index horizon = u.rows();
  if (horizon < 2) return;
  u.topRows(horizon - 1) = u.bottomRows(horizon - 1).eval();
  u.row(horizon - 1) = u.row(horizon - 2);
}

}  // namespace mppi_detail

template <typename Model, typename Cost,
          typename Sampler = GaussianSampler<Model::kControlDim>>
class Mppi {
 public:
  static constexpr int kStateDim = Model::kStateDim;
  static constexpr int kControlDim = Model::kControlDim;

  using State = Eigen::Matrix<double, kStateDim, 1>;
  using Control = Eigen::Matrix<double, kControlDim, 1>;
  // row t = control at step t
  using ControlSequence = Eigen::Matrix<double, Eigen::Dynamic, kControlDim>;

  struct Params {
    int num_samples = 1000;
    int horizon_steps = 40;
    double dt = 0.05;

    // inverse temperature: divides cost differences in the weights
    double lambda = 1.0;
    // alpha in [0,1] of Williams 2018: gamma = lambda * (1 - alpha) scales
    // the importance-sampling control-cost term. alpha = 1 disables the pull
    // toward the base distribution entirely.
    double control_cost_decoupling = 0.5;

    // per-channel sampling standard deviation (must be > 0 on every channel:
    // Sigma^{-1} appears in the control-cost term)
    Control sigma = Control::Ones();

    // actuator box constraints applied to every sampled control
    Control u_min =
        Control::Constant(-std::numeric_limits<double>::infinity());
    Control u_max = Control::Constant(std::numeric_limits<double>::infinity());

    std::uint64_t seed = 42;
  };

  Mppi(Model model, Cost cost, const Params &params)
      : model_(model),
        cost_(cost),
        params_(params),
        sampler_(params.seed),
        u_(ControlSequence::Zero(params.horizon_steps, kControlDim)),
        noise_(static_cast<std::size_t>(params.num_samples),
               ControlSequence::Zero(params.horizon_steps, kControlDim)),
        costs_(params.num_samples),
        weights_(params.num_samples),
        sigma_inv_sq_(params.sigma.cwiseProduct(params.sigma).cwiseInverse()) {}

  // One MPPI iteration from state x0: shift, sample, roll out, reweight.
  // Returns the updated control sequence (row 0 = the command to execute).
  const ControlSequence &Plan(const State &x0) {
    mppi_detail::ShiftSequence(u_);
    sampler_.SampleNoise(noise_, params_.sigma);

    const double gamma =
        params_.lambda * (1.0 - params_.control_cost_decoupling);

    for (int k = 0; k < params_.num_samples; ++k) {
      const ControlSequence &eps = noise_[static_cast<std::size_t>(k)];
      State x = x0;
      double cost = 0.0;
      for (int t = 0; t < params_.horizon_steps; ++t) {
        Control v = u_.row(t).transpose() + eps.row(t).transpose();
        v = v.cwiseMax(params_.u_min).cwiseMin(params_.u_max);
        x = model_.Step(x, v, params_.dt);
        cost += cost_.StageCost(x, v, t);
        // importance-sampling correction (eq. corrected-cost in the note)
        cost += gamma *
                (u_.row(t).transpose().cwiseProduct(sigma_inv_sq_).dot(
                    eps.row(t).transpose()));
      }
      cost += cost_.TerminalCost(x);
      costs_(k) = cost;
    }

    mppi_detail::SoftmaxWeights(costs_, params_.lambda, weights_);
    last_best_cost_ = costs_.minCoeff();
    last_ess_ = 1.0 / weights_.squaredNorm();

    for (int k = 0; k < params_.num_samples; ++k) {
      if (k == 0) {
        u_delta_ = weights_(k) * noise_[static_cast<std::size_t>(k)];
      } else {
        u_delta_ += weights_(k) * noise_[static_cast<std::size_t>(k)];
      }
    }
    u_ += u_delta_;
    // the executed command must respect the box constraints too
    for (Eigen::Index t = 0; t < u_.rows(); ++t) {
      u_.row(t) = u_.row(t)
                      .transpose()
                      .cwiseMax(params_.u_min)
                      .cwiseMin(params_.u_max)
                      .transpose();
    }
    return u_;
  }

  Control Command() const { return u_.row(0).transpose(); }
  const ControlSequence &Sequence() const { return u_; }

  void Reset() { u_.setZero(); }

  // diagnostics (telemetry hooks)
  double LastBestCost() const { return last_best_cost_; }
  // effective sample size in [1, K]: near 1 means weight collapse
  double LastEffectiveSampleSize() const { return last_ess_; }

 private:
  Model model_;
  Cost cost_;
  Params params_;
  Sampler sampler_;

  ControlSequence u_;
  ControlSequence u_delta_;
  std::vector<ControlSequence> noise_;
  Eigen::VectorXd costs_;
  Eigen::VectorXd weights_;
  Control sigma_inv_sq_;

  double last_best_cost_ = 0.0;
  double last_ess_ = 0.0;
};

}  // namespace xmotion

#endif  // XMNAV_MPPI_MPPI_HPP
