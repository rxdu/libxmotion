/*
 * @file rollout_config.hpp
 * @brief Scalar launch configuration shared by the CUDA rollout kernels.
 *
 * Compiled by nvcc: keep the include surface minimal (no Eigen/STL beyond
 * <type_traits>).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MPPI_CUDA_ROLLOUT_CONFIG_HPP
#define XMNAV_MPPI_CUDA_ROLLOUT_CONFIG_HPP

#include <type_traits>

namespace xmotion {

// one per Evaluate() call; sigma is used only by the device-sampling path
template <int StateDim, int ControlDim>
struct RolloutConfig {
  float x0[StateDim];
  float u_min[ControlDim];
  float u_max[ControlDim];
  float sigma_inv_sq[ControlDim];
  float sigma[ControlDim];
  float dt;
  float gamma;
  int num_samples;
  int horizon;
};

static_assert(std::is_trivially_copyable<RolloutConfig<13, 12>>::value,
              "kernel argument must be trivially copyable");

}  // namespace xmotion

#endif  // XMNAV_MPPI_CUDA_ROLLOUT_CONFIG_HPP
