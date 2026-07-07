/*
 * @file kernels.hpp
 * @brief Launch wrappers for the CUDA rollout kernels.
 *
 * The generic rollout kernel is templated on the device program; the .cu
 * translation unit (the only one nvcc compiles) instantiates it for each
 * program type through these overloads. All pointers are device pointers;
 * every wrapper returns 0 on success or a cudaError_t value — the host
 * backends translate failures into exceptions.
 *
 * Compiled by both nvcc and the host compiler: minimal include surface.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MPPI_CUDA_KERNELS_HPP
#define XMNAV_MPPI_CUDA_KERNELS_HPP

#include "xmnav/mppi/cuda/rollout_config.hpp"
#include "xmnav/mppi/cuda/srb_program.hpp"
#include "xmnav/mppi/cuda/wheeled_program.hpp"

namespace xmotion {

// upload-path rollouts: host-generated noise already in d_noise
int LaunchRollouts(const WheeledProgram *d_prog,
                   const RolloutConfig<3, 2> &cfg, const float *d_u,
                   const float *d_noise, float *d_costs);
int LaunchRollouts(const SrbProgram *d_prog,
                   const RolloutConfig<13, 12> &cfg, const float *d_u,
                   const float *d_noise, float *d_costs);

// device-sampling rollouts: each thread draws its own Gaussian noise
// (Philox stream (seed, sample index, offset)) and writes it to d_noise
// for the weighted update / introspection
int LaunchSampledRollouts(const WheeledProgram *d_prog,
                          const RolloutConfig<3, 2> &cfg,
                          unsigned long long seed, unsigned long long offset,
                          const float *d_u, float *d_noise, float *d_costs);

// u_delta[e] = sum_k weights[k] * noise[k][e] over the flattened
// horizon*control_dim elements
int LaunchWeightedUpdate(const float *d_noise, const float *d_weights,
                         int num_samples, int elems_per_sample,
                         float *d_u_delta);

}  // namespace xmotion

#endif  // XMNAV_MPPI_CUDA_KERNELS_HPP
