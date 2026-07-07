/*
 * @file cuda_rollout_backend.hpp
 * @brief CUDA rollout backend for the wheeled MPPI configuration.
 *
 * Implements the rollout-backend seam of mppi.hpp on the GPU: one CUDA
 * thread per sample evaluates the WheeledProgram (the same model/cost cores
 * the CPU backend inlines) in float32 — FP64 runs at 1/32 rate on GTX-class
 * and Jetson Orin hardware, and sampling MPC does not need double rollouts
 * (MPPI-Generic ships float for the same reason). The seam stays
 * double-typed: inputs are narrowed on upload, costs widened on download,
 * so CPU and GPU costs agree to float accumulation error (bounded by test),
 * not bitwise.
 *
 * Interface is host-only C++ (device code lives in the .cu); build with
 * -DXMNAV_WITH_CUDA=ON and link xmotion::mppi_cuda. Construction and
 * Evaluate() throw std::runtime_error on CUDA failures — no silent
 * degradation; callers choose the CPU backend when CudaDeviceAvailable()
 * is false.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MPPI_CUDA_CUDA_ROLLOUT_BACKEND_HPP
#define XMNAV_MPPI_CUDA_CUDA_ROLLOUT_BACKEND_HPP

#include <memory>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "xmnav/mppi/critics.hpp"
#include "xmnav/mppi/models/diff_drive.hpp"
#include "xmnav/mppi/mppi.hpp"

namespace xmotion {

// true if at least one CUDA device is usable (false also when the runtime
// itself is unavailable)
bool CudaDeviceAvailable();

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
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace xmotion

#endif  // XMNAV_MPPI_CUDA_CUDA_ROLLOUT_BACKEND_HPP
