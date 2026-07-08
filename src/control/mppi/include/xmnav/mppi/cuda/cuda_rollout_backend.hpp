/*
 * @file cuda_rollout_backend.hpp
 * @brief CUDA rollout backends for the MPPI controller.
 *
 * Three backends implementing the rollout-backend seam of mppi.hpp on the
 * GPU, all computing in float32 (FP64 runs at 1/32 rate on GTX-class and
 * Jetson Orin hardware; sampling MPC ships float — MPPI-Generic does the
 * same). The seam stays double-typed: inputs are narrowed on upload, costs
 * widened on download, so CPU and GPU costs agree to float accumulation
 * error (bounded by test), not bitwise.
 *
 * - CudaWheeledRolloutBackend: diff-drive + Se2 goal/obstacle costs;
 *   host-generated noise uploaded per plan (exact-equivalence testable
 *   against the CPU backend).
 * - CudaSrbRolloutBackend: SRB quadruped + tracking/cone/regulation costs;
 *   per-step foot plan and contact schedule shipped in the device program.
 *   Control regularization must be diagonal (validated).
 * - CudaWheeledSamplingBackend: as the wheeled backend, but Gaussian noise
 *   is drawn on-device from per-sample Philox streams (kGeneratesNoise —
 *   Mppi skips its host sampler) and the weighted update runs on-device,
 *   removing the K-sized host work and transfers. Deterministic for a
 *   given seed, but its noise realizations differ from any host sampler's.
 *
 * Interface is host-only C++ (device code lives in the .cu); build with
 * -DXMNAV_WITH_CUDA=ON and link xmotion::mppi_cuda. Construction and
 * Evaluate() throw std::runtime_error on CUDA failures — no silent
 * degradation; callers choose a CPU backend when CudaDeviceAvailable()
 * is false.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MPPI_CUDA_CUDA_ROLLOUT_BACKEND_HPP
#define XMNAV_MPPI_CUDA_CUDA_ROLLOUT_BACKEND_HPP

#include <cstdint>
#include <memory>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "xmnav/mppi/critics.hpp"
#include "xmnav/mppi/critics_srb.hpp"
#include "xmnav/models/diff_drive.hpp"
#include "xmnav/models/srb_quadruped.hpp"
#include "xmnav/mppi/mppi.hpp"

namespace xmotion {

// true if at least one CUDA device is usable (false also when the runtime
// itself is unavailable)
bool CudaDeviceAvailable();

namespace cuda_detail {
struct DeviceWorkspace;  // pinned/device buffer pairs, defined in the .cpp
}

class CudaWheeledRolloutBackend {
 public:
  static constexpr int kStateDim = 3;
  static constexpr int kControlDim = 2;

  using Model = DiffDriveModel;
  using Cost = CompositeCost<Se2GoalCost, CircularObstacleCost>;
  using State = Eigen::Matrix<double, kStateDim, 1>;
  using Control = Eigen::Matrix<double, kControlDim, 1>;
  using ControlSequence = Eigen::Matrix<double, Eigen::Dynamic, kControlDim>;
  using Params = MppiParams<kControlDim>;

  CudaWheeledRolloutBackend();
  ~CudaWheeledRolloutBackend();

  CudaWheeledRolloutBackend(const CudaWheeledRolloutBackend &) = delete;
  CudaWheeledRolloutBackend &operator=(const CudaWheeledRolloutBackend &) =
      delete;
  CudaWheeledRolloutBackend(CudaWheeledRolloutBackend &&) noexcept;
  CudaWheeledRolloutBackend &operator=(CudaWheeledRolloutBackend &&) noexcept;

  // Rollout-backend seam (same signature the CPU backend implements). The
  // device program is rebuilt from the live cost object on every call, so
  // critic mutations between Plan() calls behave as on the CPU.
  void Evaluate(const Model &model, const Cost &cost, const Params &params,
                double gamma, const State &x0, const ControlSequence &u,
                const std::vector<ControlSequence> &noise,
                const Control &sigma_inv_sq, Eigen::VectorXd &costs);

 private:
  std::unique_ptr<cuda_detail::DeviceWorkspace> ws_;
};

class CudaSrbRolloutBackend {
 public:
  static constexpr int kStateDim = 13;
  static constexpr int kControlDim = 12;

  using Model = SrbQuadrupedModel;
  using Cost = CompositeCost<SrbTrackingCost, FrictionConeCost,
                             QuadraticControlCost<13, 12>>;
  using State = Eigen::Matrix<double, kStateDim, 1>;
  using Control = Eigen::Matrix<double, kControlDim, 1>;
  using ControlSequence = Eigen::Matrix<double, Eigen::Dynamic, kControlDim>;
  using Params = MppiParams<kControlDim>;

  CudaSrbRolloutBackend();
  ~CudaSrbRolloutBackend();

  CudaSrbRolloutBackend(const CudaSrbRolloutBackend &) = delete;
  CudaSrbRolloutBackend &operator=(const CudaSrbRolloutBackend &) = delete;
  CudaSrbRolloutBackend(CudaSrbRolloutBackend &&) noexcept;
  CudaSrbRolloutBackend &operator=(CudaSrbRolloutBackend &&) noexcept;

  // The device program is rebuilt from the live model context (per-step
  // foot plan + contact schedule) and cost object on every call — the
  // per-cycle SetContext() pattern behaves exactly as on the CPU.
  // Validated fail-loud: horizon <= SrbProgram::kMaxHorizon, diagonal R,
  // and the friction-cone schedule must agree with the model's.
  void Evaluate(const Model &model, const Cost &cost, const Params &params,
                double gamma, const State &x0, const ControlSequence &u,
                const std::vector<ControlSequence> &noise,
                const Control &sigma_inv_sq, Eigen::VectorXd &costs);

 private:
  std::unique_ptr<cuda_detail::DeviceWorkspace> ws_;
};

class CudaWheeledSamplingBackend {
 public:
  // Mppi skips its host sampler and delegates noise generation, the
  // weighted update, and candidate-noise access to this backend
  static constexpr bool kGeneratesNoise = true;

  static constexpr int kStateDim = 3;
  static constexpr int kControlDim = 2;

  using Model = DiffDriveModel;
  using Cost = CompositeCost<Se2GoalCost, CircularObstacleCost>;
  using State = Eigen::Matrix<double, kStateDim, 1>;
  using Control = Eigen::Matrix<double, kControlDim, 1>;
  using ControlSequence = Eigen::Matrix<double, Eigen::Dynamic, kControlDim>;
  using Params = MppiParams<kControlDim>;

  explicit CudaWheeledSamplingBackend(std::uint64_t seed);
  ~CudaWheeledSamplingBackend();

  CudaWheeledSamplingBackend(const CudaWheeledSamplingBackend &) = delete;
  CudaWheeledSamplingBackend &operator=(const CudaWheeledSamplingBackend &) =
      delete;
  CudaWheeledSamplingBackend(CudaWheeledSamplingBackend &&) noexcept;
  CudaWheeledSamplingBackend &operator=(
      CudaWheeledSamplingBackend &&) noexcept;

  // seam signature; the host `noise` argument is ignored (and left
  // untouched) — noise is drawn on-device and retained there
  void Evaluate(const Model &model, const Cost &cost, const Params &params,
                double gamma, const State &x0, const ControlSequence &u,
                const std::vector<ControlSequence> &noise,
                const Control &sigma_inv_sq, Eigen::VectorXd &costs);

  // u_delta = sum_k weights(k) * eps_k, computed on-device against the
  // noise of the last Evaluate(); u_delta is resized to (horizon x 2)
  void ApplyWeightedUpdate(const Eigen::VectorXd &weights,
                           ControlSequence &u_delta);

  // download one sample's noise from the last Evaluate() (introspection)
  void DownloadNoiseSample(int k, ControlSequence &eps) const;

 private:
  std::unique_ptr<cuda_detail::DeviceWorkspace> ws_;
  std::uint64_t seed_;
  unsigned long long stream_offset_ = 0;
  int last_num_samples_ = 0;
  int last_horizon_ = 0;
};

}  // namespace xmotion

#endif  // XMNAV_MPPI_CUDA_CUDA_ROLLOUT_BACKEND_HPP
