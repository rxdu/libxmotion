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
 * Design: the controller is templated on four seams —
 *   Model:   static kStateDim/kControlDim; State Step(State, Control, t, dt)
 *   Cost:    double StageCost(state, control, t); double TerminalCost(state)
 *   Sampler: void SampleNoise(noise_buffers, sigma)
 *   Backend: void Evaluate(model, cost, params, gamma, x0, u, noise,
 *            sigma_inv_sq, costs) — the per-sample rollout phase; CPU
 *            serial/threaded (rollout_backend.hpp) or CUDA (cuda/). A
 *            backend declaring kGeneratesNoise = true samples on its own
 *            device: Plan() then skips the host sampler and delegates the
 *            weighted update and candidate-noise access to it.
 * The hot path (Plan) performs no heap allocation: all rollout buffers are
 * sized at construction. Sampling is deterministic under a fixed seed, and
 * backends are required to be deterministic too (the CPU backends bitwise
 * for any thread count; device-sampling backends per seed).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MPPI_MPPI_HPP
#define XMNAV_MPPI_MPPI_HPP

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "xmbase/telemetry/telemetry.hpp"
#include "xmnav/mppi/rollout_backend.hpp"
#include "xmnav/mppi/sampler.hpp"

namespace xmotion {

// Introspection snapshot of one Plan() call: a decimated set of candidate
// rollouts (top-weighted + a uniform subsample), their costs/weights, and
// the updated nominal trajectory. Produced only when introspection is
// enabled; see docs/typst/mppi.typ.
template <int StateDim, int ControlDim>
struct MppiSnapshot {
  struct Candidate {
    double cost = 0.0;
    double weight = 0.0;
    // rollout states, row t (T rows: the states AFTER each step)
    Eigen::Matrix<double, Eigen::Dynamic, StateDim> states;
    // the clamped controls that produced them
    Eigen::Matrix<double, Eigen::Dynamic, ControlDim> controls;
  };
  std::vector<Candidate> candidates;
  // states of the updated (executed) nominal sequence
  Eigen::Matrix<double, Eigen::Dynamic, StateDim> nominal_states;
  Eigen::Matrix<double, Eigen::Dynamic, ControlDim> nominal_controls;
  double effective_sample_size = 0.0;
  double best_cost = 0.0;
  std::uint64_t plan_index = 0;
};

}  // namespace xmotion

namespace xmotion {
namespace mppi_detail {
// selection for introspection: indices of the top_n lowest-cost samples
// plus an even stride over the rest
inline void SelectCandidates(const Eigen::VectorXd &costs, int top_n,
                             int subsample_n, std::vector<int> &out) {
  const int k = static_cast<int>(costs.size());
  out.clear();
  std::vector<int> order(static_cast<std::size_t>(k));
  for (int i = 0; i < k; ++i) order[static_cast<std::size_t>(i)] = i;
  const int n = std::min(top_n, k);
  std::partial_sort(order.begin(), order.begin() + n, order.end(),
                    [&](int a, int b) { return costs(a) < costs(b); });
  out.assign(order.begin(), order.begin() + n);
  const int stride = std::max(1, k / std::max(1, subsample_n));
  for (int i = 0; i < k && static_cast<int>(out.size()) < n + subsample_n;
       i += stride) {
    out.push_back(i);
  }
}


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
  // in-place row-wise shift: copying row t <- t+1 in ascending order has
  // no aliasing hazard, unlike the overlapping block assignment whose
  // .eval() would heap-allocate a temporary every Plan()
  for (Eigen::Index t = 0; t + 1 < horizon; ++t) u.row(t) = u.row(t + 1);
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

// Backends that generate their own noise on their compute device declare
// `static constexpr bool kGeneratesNoise = true`: Plan() then skips the
// host sampler, delegates the weighted update to the backend, and fetches
// candidate noise through DownloadNoiseSample() for introspection.
template <typename Backend, typename = void>
struct BackendGeneratesNoise : std::false_type {};
template <typename Backend>
struct BackendGeneratesNoise<Backend,
                             std::enable_if_t<Backend::kGeneratesNoise>>
    : std::true_type {};

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
          typename Sampler = GaussianSampler<Model::kControlDim>,
          typename Backend = CpuRolloutBackend<Model, Cost>>
class Mppi {
 public:
  static constexpr int kStateDim = Model::kStateDim;
  static constexpr int kControlDim = Model::kControlDim;

  using State = Eigen::Matrix<double, kStateDim, 1>;
  using Control = Eigen::Matrix<double, kControlDim, 1>;
  // row t = control at step t
  using ControlSequence = Eigen::Matrix<double, Eigen::Dynamic, kControlDim>;

  using Params = MppiParams<kControlDim>;
  using Snapshot = MppiSnapshot<kStateDim, kControlDim>;


  Mppi(Model model, Cost cost, const Params &params)
      : Mppi(model, cost, params, Sampler(params.seed)) {}

  // for samplers with configuration beyond the seed (colored noise, spline
  // knots, log-MPPI)
  Mppi(Model model, Cost cost, const Params &params, Sampler sampler)
      : Mppi(model, cost, params, std::move(sampler), Backend{}) {}

  // for configured rollout backends (threaded CPU pool, CUDA)
  Mppi(Model model, Cost cost, const Params &params, Sampler sampler,
       Backend backend)
      : model_(model),
        cost_(cost),
        params_(params),
        sampler_(std::move(sampler)),
        backend_(std::move(backend)),
        u_(ControlSequence::Zero(params.horizon_steps, kControlDim)),
        noise_(mppi_detail::BackendGeneratesNoise<Backend>::value
                   ? 0  // noise lives on the backend's device
                   : static_cast<std::size_t>(params.num_samples),
               ControlSequence::Zero(params.horizon_steps, kControlDim)),
        costs_(params.num_samples),
        weights_(params.num_samples),
        sigma_inv_sq_(params.sigma.cwiseProduct(params.sigma).cwiseInverse()) {}

  // One MPPI iteration from state x0: shift, sample, roll out, reweight.
  // Returns the updated control sequence (row 0 = the command to execute).
  const ControlSequence &Plan(const State &x0) {
    XM_SPAN("control.mppi.plan");
    mppi_detail::ShiftSequence(u_);
    if constexpr (!mppi_detail::BackendGeneratesNoise<Backend>::value) {
      sampler_.SampleNoise(noise_, params_.sigma);
    }

    const double gamma =
        params_.lambda * (1.0 - params_.control_cost_decoupling);

    // per-sample rollout + cost accumulation lives in the backend (the
    // parallel phase); everything below is the serial reduction
    backend_.Evaluate(model_, cost_, params_, gamma, x0, u_, noise_,
                      sigma_inv_sq_, costs_);

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

    if (introspection_enabled_) {
      CaptureCandidates(x0);
    }

    if constexpr (mppi_detail::BackendGeneratesNoise<Backend>::value) {
      backend_.ApplyWeightedUpdate(weights_, u_delta_);
    } else {
      for (int k = 0; k < params_.num_samples; ++k) {
        if (k == 0) {
          u_delta_ = weights_(k) * noise_[static_cast<std::size_t>(k)];
        } else {
          u_delta_ += weights_(k) * noise_[static_cast<std::size_t>(k)];
        }
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
    if (introspection_enabled_) {
      RolloutStates(x0, u_, snapshot_.nominal_states);
      snapshot_.nominal_controls = u_;
      snapshot_.effective_sample_size = last_ess_;
      snapshot_.best_cost = last_best_cost_;
      ++snapshot_.plan_index;
    }
    return u_;
  }

  Control Command() const { return u_.row(0).transpose(); }
  // mutable access for models carrying per-cycle context (contact schedules,
  // foot positions) that the application updates before each Plan()
  Model &model() { return model_; }
  const ControlSequence &Sequence() const { return u_; }

  void Reset() { u_.setZero(); }

  // Introspection: capture a decimated snapshot of each Plan() — the top_n
  // best candidates plus subsample_n evenly-strided ones, with their costs,
  // weights, and full state rollouts, and the updated nominal trajectory.
  // Cost when enabled: (top_n + subsample_n + 1) extra rollouts per plan
  // (e.g. 33/2048 = 1.6%); zero when disabled (a branch).
  void EnableIntrospection(int top_n = 16, int subsample_n = 16) {
    introspect_top_n_ = top_n;
    introspect_sub_n_ = subsample_n;
    snapshot_.candidates.reserve(
        static_cast<std::size_t>(top_n + subsample_n));
    introspection_enabled_ = true;
  }
  void DisableIntrospection() { introspection_enabled_ = false; }
  // valid after Plan() when introspection is enabled
  const Snapshot &LastSnapshot() const { return snapshot_; }
  // seed every step of the nominal sequence (e.g. gravity-compensating
  // stance forces) — the standard warm start for force-space sampling
  void SeedSequence(const Control &u0) { u_ = u0.transpose().replicate(u_.rows(), 1); }

  // diagnostics (telemetry hooks)
  double LastBestCost() const { return last_best_cost_; }
  // effective sample size in [1, K]: near 1 means weight collapse
  double LastEffectiveSampleSize() const { return last_ess_; }

  // Live-tuning setters (interactive tuner, adaptive schemes). Both are
  // cheap and allocation-free but must be called from the planning thread
  // between Plan() invocations — relay values through atomics if the knobs
  // are driven from a UI thread.
  void SetTemperature(double lambda) {
    if (!(lambda > 0.0)) {
      throw std::invalid_argument("MPPI temperature lambda must be > 0");
    }
    params_.lambda = lambda;
  }
  double Temperature() const { return params_.lambda; }
  // updates the cached Sigma^{-1} used by the control-cost term with it
  void SetSigma(const Control &sigma) {
    if (!(sigma.minCoeff() > 0.0)) {
      throw std::invalid_argument("MPPI sigma must be > 0 on every channel");
    }
    params_.sigma = sigma;
    sigma_inv_sq_ = sigma.cwiseProduct(sigma).cwiseInverse();
  }
  Control Sigma() const { return params_.sigma; }

 private:
  // noise of one sample for introspection re-rolls: host-side buffer, or
  // fetched from the backend's device when it generates the noise
  const ControlSequence &CandidateNoise(int k) {
    if constexpr (mppi_detail::BackendGeneratesNoise<Backend>::value) {
      backend_.DownloadNoiseSample(k, candidate_noise_scratch_);
      return candidate_noise_scratch_;
    } else {
      return noise_[static_cast<std::size_t>(k)];
    }
  }

  // roll a control sequence (clamped) and record the post-step states
  void RolloutStates(const State &x0, const ControlSequence &seq,
                     Eigen::Matrix<double, Eigen::Dynamic, kStateDim> &out,
                     Eigen::Matrix<double, Eigen::Dynamic, kControlDim>
                         *out_controls = nullptr) {
    out.resize(seq.rows(), kStateDim);
    if (out_controls != nullptr) out_controls->resize(seq.rows(), kControlDim);
    State x = x0;
    for (Eigen::Index t = 0; t < seq.rows(); ++t) {
      Control v = seq.row(t).transpose();
      v = v.cwiseMax(params_.u_min).cwiseMin(params_.u_max);
      x = model_.Step(x, v, static_cast<int>(t), params_.dt);
      out.row(t) = x.transpose();
      if (out_controls != nullptr) out_controls->row(t) = v.transpose();
    }
  }

  // re-roll the selected candidates (perturbed sequences around the
  // pre-update nominal) — states are exact re-computations, so recorded
  // trajectories are consistent with what the optimizer scored
  void CaptureCandidates(const State &x0) {
    mppi_detail::SelectCandidates(costs_, introspect_top_n_,
                                  introspect_sub_n_, selected_);
    snapshot_.candidates.resize(selected_.size());
    for (std::size_t s = 0; s < selected_.size(); ++s) {
      const int k = selected_[s];
      auto &cand = snapshot_.candidates[s];
      cand.cost = costs_(k);
      cand.weight = weights_(k);
      RolloutStates(x0, u_ + CandidateNoise(k), cand.states, &cand.controls);
    }
  }

  Model model_;
  Cost cost_;
  Params params_;
  Sampler sampler_;
  Backend backend_;

  ControlSequence u_;
  ControlSequence u_delta_;
  std::vector<ControlSequence> noise_;
  Eigen::VectorXd costs_;
  Eigen::VectorXd weights_;
  Control sigma_inv_sq_;

  double last_best_cost_ = 0.0;
  double last_ess_ = 0.0;

  bool introspection_enabled_ = false;
  int introspect_top_n_ = 0;
  int introspect_sub_n_ = 0;
  std::vector<int> selected_;
  Snapshot snapshot_;
  ControlSequence candidate_noise_scratch_;

  // pre-acquired telemetry handles (atomic slot writes, wait-free; no-ops
  // when no telemetry binding is installed)
  telemetry::Gauge ess_gauge_ =
      telemetry::GetGauge("control.mppi.effective_sample_size");
  telemetry::Gauge best_cost_gauge_ =
      telemetry::GetGauge("control.mppi.best_cost");
};

}  // namespace xmotion

#endif  // XMNAV_MPPI_MPPI_HPP
