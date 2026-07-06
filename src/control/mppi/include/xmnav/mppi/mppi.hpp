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

#include "xmbase/telemetry/telemetry.hpp"
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

// Savitzky-Golay filter, window 5, quadratic fit: coefficients
// (-3, 12, 17, 12, -3)/35; the two samples at each boundary are left
// untouched (the head of the sequence is the command about to execute at
// the next shift, so boundary bias matters more than boundary smoothness).
template <typename Sequence>
inline void SavitzkyGolay5(Sequence &u) {
  const Eigen::Index horizon = u.rows();
  if (horizon < 5) return;
  Sequence s = u;
  for (Eigen::Index t = 2; t + 2 < horizon; ++t) {
    u.row(t) = (-3.0 * s.row(t - 2) + 12.0 * s.row(t - 1) + 17.0 * s.row(t) +
                12.0 * s.row(t + 1) - 3.0 * s.row(t + 2)) /
               35.0;
  }
}

}  // namespace mppi_detail

template <int ControlDim>
struct MppiParams {
using Control = Eigen::Matrix<double, ControlDim, 1>;

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

  // Normalize (S - rho) by the sample spread (mean - min) before the
  // softmax, making lambda scale-free in the cost magnitude. Without it,
  // lambda must be re-tuned to the per-horizon cost scale and easily
  // degenerates into winner-take-all (effective sample size -> 1).
  bool normalize_cost_spread = false;

  // Savitzky-Golay smoothing (window 5, quadratic) applied to the updated
  // control sequence. Cheap chattering mitigation (the Nav2 approach); for
  // smoothness by construction prefer SplineKnotSampler or
  // ColoredNoiseSampler.
  bool smooth_output = false;
};
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

  using Params = MppiParams<kControlDim>;


  Mppi(Model model, Cost cost, const Params &params)
      : Mppi(model, cost, params, Sampler(params.seed)) {}

  // for samplers with configuration beyond the seed (colored noise, spline
  // knots, log-MPPI)
  Mppi(Model model, Cost cost, const Params &params, Sampler sampler)
      : model_(model),
        cost_(cost),
        params_(params),
        sampler_(std::move(sampler)),
        u_(ControlSequence::Zero(params.horizon_steps, kControlDim)),
        noise_(static_cast<std::size_t>(params.num_samples),
               ControlSequence::Zero(params.horizon_steps, kControlDim)),
        costs_(params.num_samples),
        weights_(params.num_samples),
        sigma_inv_sq_(params.sigma.cwiseProduct(params.sigma).cwiseInverse()) {}

  // One MPPI iteration from state x0: shift, sample, roll out, reweight.
  // Returns the updated control sequence (row 0 = the command to execute).
  const ControlSequence &Plan(const State &x0) {
    XM_SPAN("control.mppi.plan");
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

    double lambda = params_.lambda;
    if (params_.normalize_cost_spread) {
      const double spread = costs_.mean() - costs_.minCoeff();
      if (spread > 1e-12) lambda = params_.lambda * spread;
    }
    mppi_detail::SoftmaxWeights(costs_, lambda, weights_);
    last_best_cost_ = costs_.minCoeff();
    last_ess_ = 1.0 / weights_.squaredNorm();
    ess_gauge_.Set(last_ess_);
    best_cost_gauge_.Set(last_best_cost_);

    for (int k = 0; k < params_.num_samples; ++k) {
      if (k == 0) {
        u_delta_ = weights_(k) * noise_[static_cast<std::size_t>(k)];
      } else {
        u_delta_ += weights_(k) * noise_[static_cast<std::size_t>(k)];
      }
    }
    u_ += u_delta_;
    if (params_.smooth_output) {
      mppi_detail::SavitzkyGolay5(u_);
    }
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

  // pre-acquired telemetry handles (atomic slot writes, wait-free; no-ops
  // when no telemetry binding is installed)
  telemetry::Gauge ess_gauge_ =
      telemetry::GetGauge("control.mppi.effective_sample_size");
  telemetry::Gauge best_cost_gauge_ =
      telemetry::GetGauge("control.mppi.best_cost");
};

}  // namespace xmotion

#endif  // XMNAV_MPPI_MPPI_HPP
