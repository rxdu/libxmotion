/*
 * cuda_rollout_backend.cu
 *
 * Device side of the CUDA rollout backends: the kernels and their launch
 * wrappers, nothing else — all host logic lives in cuda_rollout_backend.cpp
 * so nvcc never sees Eigen or heavyweight host headers (nvcc 11.x cannot
 * parse post-11.2 libstdc++ <functional>, and the split keeps that class
 * of problem structurally impossible).
 *
 * Kernel layout: one thread per sample, each rolling its perturbed control
 * sequence forward and accumulating stage + importance-sampling costs —
 * the "split rollout" arrangement of MPPI-Generic, the right shape for
 * models whose state fits in registers (13 floats for the SRB quadruped).
 * The generic kernel is templated on the device program; the sampling
 * variant additionally draws its noise from a per-sample Philox stream
 * (counter-based: deterministic for a given (seed, sample, plan offset),
 * independent of thread scheduling).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <curand_kernel.h>

#include <cstddef>

#include "xmnav/mppi/cuda/kernels.hpp"

namespace xmotion {

namespace {

template <typename Program>
__global__ void RolloutKernel(
    const Program *__restrict__ prog,
    RolloutConfig<Program::kStateDim, Program::kControlDim> cfg,
    const float *__restrict__ u, const float *__restrict__ noise,
    float *__restrict__ costs) {
  constexpr int SD = Program::kStateDim;
  constexpr int CD = Program::kControlDim;
  const int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= cfg.num_samples) return;

  const float *eps =
      noise + static_cast<std::size_t>(k) * cfg.horizon * CD;
  float x[SD];
#pragma unroll
  for (int i = 0; i < SD; ++i) x[i] = cfg.x0[i];
  float cost = 0.0f;
  for (int t = 0; t < cfg.horizon; ++t) {
    float v[CD];
    float is_term = 0.0f;
#pragma unroll
    for (int j = 0; j < CD; ++j) {
      const float e = eps[t * CD + j];
      v[j] = fminf(fmaxf(u[t * CD + j] + e, cfg.u_min[j]), cfg.u_max[j]);
      // importance-sampling correction, same form as the CPU backend
      is_term += u[t * CD + j] * cfg.sigma_inv_sq[j] * e;
    }
    float next[SD];
    prog->Step(x, v, t, cfg.dt, next);
#pragma unroll
    for (int i = 0; i < SD; ++i) x[i] = next[i];
    cost += prog->StageCost(x, v, t) + cfg.gamma * is_term;
  }
  cost += prog->TerminalCost(x);
  costs[k] = cost;
}

// as above, but each thread generates its own noise and records it
template <typename Program>
__global__ void SampledRolloutKernel(
    const Program *__restrict__ prog,
    RolloutConfig<Program::kStateDim, Program::kControlDim> cfg,
    unsigned long long seed, unsigned long long offset,
    const float *__restrict__ u, float *__restrict__ noise,
    float *__restrict__ costs) {
  constexpr int SD = Program::kStateDim;
  constexpr int CD = Program::kControlDim;
  const int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= cfg.num_samples) return;

  curandStatePhilox4_32_10_t rng;
  curand_init(seed, static_cast<unsigned long long>(k), offset, &rng);

  float *eps = noise + static_cast<std::size_t>(k) * cfg.horizon * CD;
  float x[SD];
#pragma unroll
  for (int i = 0; i < SD; ++i) x[i] = cfg.x0[i];
  float cost = 0.0f;
  for (int t = 0; t < cfg.horizon; ++t) {
    float v[CD];
    float is_term = 0.0f;
#pragma unroll
    for (int j = 0; j < CD; ++j) {
      const float e = curand_normal(&rng) * cfg.sigma[j];
      eps[t * CD + j] = e;
      v[j] = fminf(fmaxf(u[t * CD + j] + e, cfg.u_min[j]), cfg.u_max[j]);
      is_term += u[t * CD + j] * cfg.sigma_inv_sq[j] * e;
    }
    float next[SD];
    prog->Step(x, v, t, cfg.dt, next);
#pragma unroll
    for (int i = 0; i < SD; ++i) x[i] = next[i];
    cost += prog->StageCost(x, v, t) + cfg.gamma * is_term;
  }
  cost += prog->TerminalCost(x);
  costs[k] = cost;
}

__global__ void WeightedUpdateKernel(const float *__restrict__ noise,
                                     const float *__restrict__ weights,
                                     int num_samples, int elems_per_sample,
                                     float *__restrict__ u_delta) {
  const int e = blockIdx.x * blockDim.x + threadIdx.x;
  if (e >= elems_per_sample) return;
  float acc = 0.0f;
  for (int k = 0; k < num_samples; ++k) {
    acc += weights[k] *
           noise[static_cast<std::size_t>(k) * elems_per_sample + e];
  }
  u_delta[e] = acc;
}

constexpr int kBlock = 128;

inline int GridFor(int n) { return (n + kBlock - 1) / kBlock; }

template <typename Program>
int LaunchRolloutsImpl(
    const Program *d_prog,
    const RolloutConfig<Program::kStateDim, Program::kControlDim> &cfg,
    const float *d_u, const float *d_noise, float *d_costs) {
  RolloutKernel<Program><<<GridFor(cfg.num_samples), kBlock>>>(
      d_prog, cfg, d_u, d_noise, d_costs);
  return static_cast<int>(cudaPeekAtLastError());
}

}  // namespace

int LaunchRollouts(const WheeledProgram *d_prog,
                   const RolloutConfig<3, 2> &cfg, const float *d_u,
                   const float *d_noise, float *d_costs) {
  return LaunchRolloutsImpl(d_prog, cfg, d_u, d_noise, d_costs);
}

int LaunchRollouts(const SrbProgram *d_prog,
                   const RolloutConfig<13, 12> &cfg, const float *d_u,
                   const float *d_noise, float *d_costs) {
  return LaunchRolloutsImpl(d_prog, cfg, d_u, d_noise, d_costs);
}

int LaunchSampledRollouts(const WheeledProgram *d_prog,
                          const RolloutConfig<3, 2> &cfg,
                          unsigned long long seed, unsigned long long offset,
                          const float *d_u, float *d_noise, float *d_costs) {
  SampledRolloutKernel<WheeledProgram><<<GridFor(cfg.num_samples), kBlock>>>(
      d_prog, cfg, seed, offset, d_u, d_noise, d_costs);
  return static_cast<int>(cudaPeekAtLastError());
}

int LaunchWeightedUpdate(const float *d_noise, const float *d_weights,
                         int num_samples, int elems_per_sample,
                         float *d_u_delta) {
  WeightedUpdateKernel<<<GridFor(elems_per_sample), kBlock>>>(
      d_noise, d_weights, num_samples, elems_per_sample, d_u_delta);
  return static_cast<int>(cudaPeekAtLastError());
}

}  // namespace xmotion
