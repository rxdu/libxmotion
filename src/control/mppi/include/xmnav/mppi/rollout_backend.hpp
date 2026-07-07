/*
 * @file rollout_backend.hpp
 * @brief Rollout-evaluation backends for the MPPI controller.
 *
 * The rollout phase of MPPI — clamp the perturbed control, step the model,
 * accumulate stage + importance-sampling costs — is embarrassingly parallel
 * across samples: sample k reads shared inputs and writes only costs(k).
 * This header extracts that phase behind a backend seam (the M4 plan of
 * docs/typst/mppi.typ, after MPPI-Generic): the CUDA backend for dGPU /
 * Jetson Orin implements the same Evaluate() signature over the same
 * model/cost functors.
 *
 * CpuRolloutBackend evaluates serially by default (byte-for-byte the
 * behavior the M1 oracle tests pin down) or over a pre-allocated thread
 * pool. Chunking over samples is contiguous and the per-sample operation
 * order is unchanged, so results are bitwise identical for any thread
 * count. Thread safety requires Model::Step and the cost methods to be
 * const (all in-tree models/critics are); models carrying per-cycle
 * context (contact schedules) are updated between Plan() calls, never
 * during evaluation.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MPPI_ROLLOUT_BACKEND_HPP
#define XMNAV_MPPI_ROLLOUT_BACKEND_HPP

#include <algorithm>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <eigen3/Eigen/Dense>

namespace xmotion {

template <int ControlDim>
struct MppiParams;

namespace mppi_detail {

// Fixed set of workers dispatching a (function pointer, context) pair over
// contiguous index ranges. Function-pointer dispatch keeps Run() free of
// per-call allocation (a std::function capture could heap-allocate every
// control cycle).
class RolloutThreadPool {
 public:
  using RangeFn = void (*)(void *ctx, int begin, int end);

  explicit RolloutThreadPool(int num_threads) {
    const int n = std::max(1, num_threads);
    workers_.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
      workers_.emplace_back([this, i, n] { WorkerLoop(i, n); });
    }
  }

  RolloutThreadPool(const RolloutThreadPool &) = delete;
  RolloutThreadPool &operator=(const RolloutThreadPool &) = delete;

  ~RolloutThreadPool() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    task_cv_.notify_all();
    for (auto &w : workers_) w.join();
  }

  int num_threads() const { return static_cast<int>(workers_.size()); }

  // run fn(ctx, begin, end) over a contiguous partition of [0, total);
  // blocks until every worker finished its range
  void Run(int total, RangeFn fn, void *ctx) {
    std::unique_lock<std::mutex> lock(mutex_);
    fn_ = fn;
    ctx_ = ctx;
    total_ = total;
    pending_ = static_cast<int>(workers_.size());
    ++epoch_;
    task_cv_.notify_all();
    done_cv_.wait(lock, [this] { return pending_ == 0; });
  }

 private:
  void WorkerLoop(int index, int count) {
    std::uint64_t seen_epoch = 0;
    for (;;) {
      RangeFn fn = nullptr;
      void *ctx = nullptr;
      int total = 0;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        task_cv_.wait(lock,
                      [&] { return stop_ || epoch_ != seen_epoch; });
        if (stop_) return;
        seen_epoch = epoch_;
        fn = fn_;
        ctx = ctx_;
        total = total_;
      }
      const int begin = static_cast<int>(
          static_cast<std::int64_t>(total) * index / count);
      const int end = static_cast<int>(
          static_cast<std::int64_t>(total) * (index + 1) / count);
      if (begin < end) fn(ctx, begin, end);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (--pending_ == 0) done_cv_.notify_one();
      }
    }
  }

  std::vector<std::thread> workers_;
  std::mutex mutex_;
  std::condition_variable task_cv_;
  std::condition_variable done_cv_;
  RangeFn fn_ = nullptr;
  void *ctx_ = nullptr;
  int total_ = 0;
  int pending_ = 0;
  std::uint64_t epoch_ = 0;
  bool stop_ = false;
};

}  // namespace mppi_detail

template <typename Model, typename Cost>
class CpuRolloutBackend {
 public:
  static constexpr int kStateDim = Model::kStateDim;
  static constexpr int kControlDim = Model::kControlDim;

  using State = Eigen::Matrix<double, kStateDim, 1>;
  using Control = Eigen::Matrix<double, kControlDim, 1>;
  using ControlSequence = Eigen::Matrix<double, Eigen::Dynamic, kControlDim>;
  using Params = MppiParams<kControlDim>;

  // serial evaluation (the default): no pool, no synchronization
  CpuRolloutBackend() = default;

  // num_threads > 1 creates the worker pool once, at construction; the
  // hot path only signals it
  explicit CpuRolloutBackend(int num_threads)
      : pool_(num_threads > 1
                  ? std::make_unique<mppi_detail::RolloutThreadPool>(
                        num_threads)
                  : nullptr) {}

  // copying copies the configuration, not the workers: each backend owns
  // its own pool (keeps Mppi copyable, as it was before the seam)
  CpuRolloutBackend(const CpuRolloutBackend &other)
      : CpuRolloutBackend(other.num_threads()) {}
  CpuRolloutBackend &operator=(const CpuRolloutBackend &other) {
    if (this != &other) *this = CpuRolloutBackend(other.num_threads());
    return *this;
  }
  CpuRolloutBackend(CpuRolloutBackend &&) = default;
  CpuRolloutBackend &operator=(CpuRolloutBackend &&) = default;

  int num_threads() const { return pool_ ? pool_->num_threads() : 1; }

  // Fill costs(k) for every sample: rollout of u + noise[k] under model,
  // stage/terminal costs plus the gamma-scaled importance-sampling
  // correction (see the Plan() derivation in docs/typst/mppi.typ).
  void Evaluate(const Model &model, const Cost &cost, const Params &params,
                double gamma, const State &x0, const ControlSequence &u,
                const std::vector<ControlSequence> &noise,
                const Control &sigma_inv_sq, Eigen::VectorXd &costs) {
    EvalContext ctx{&model,  &cost,  &params,       gamma, &x0,
                    &u,      &noise, &sigma_inv_sq, &costs};
    if (pool_) {
      pool_->Run(params.num_samples, &EvalRange, &ctx);
    } else {
      EvalRange(&ctx, 0, params.num_samples);
    }
  }

 private:
  struct EvalContext {
    const Model *model;
    const Cost *cost;
    const Params *params;
    double gamma;
    const State *x0;
    const ControlSequence *u;
    const std::vector<ControlSequence> *noise;
    const Control *sigma_inv_sq;
    Eigen::VectorXd *costs;
  };

  static void EvalRange(void *raw, int begin, int end) {
    const auto &ctx = *static_cast<EvalContext *>(raw);
    for (int k = begin; k < end; ++k) {
      const ControlSequence &eps =
          (*ctx.noise)[static_cast<std::size_t>(k)];
      State x = *ctx.x0;
      double c = 0.0;
      for (int t = 0; t < ctx.params->horizon_steps; ++t) {
        Control v = ctx.u->row(t).transpose() + eps.row(t).transpose();
        v = v.cwiseMax(ctx.params->u_min).cwiseMin(ctx.params->u_max);
        x = ctx.model->Step(x, v, t, ctx.params->dt);
        c += ctx.cost->StageCost(x, v, t);
        // importance-sampling correction (eq. corrected-cost in the note)
        c += ctx.gamma *
             (ctx.u->row(t).transpose().cwiseProduct(*ctx.sigma_inv_sq).dot(
                 eps.row(t).transpose()));
      }
      c += ctx.cost->TerminalCost(x);
      (*ctx.costs)(k) = c;
    }
  }

  std::unique_ptr<mppi_detail::RolloutThreadPool> pool_;
};

}  // namespace xmotion

#endif  // XMNAV_MPPI_ROLLOUT_BACKEND_HPP
