/*
 * cuda_rollout_backend.cu
 *
 * Device side of the CUDA rollout backend: the kernel and its launch
 * wrapper, nothing else — all host logic lives in cuda_rollout_backend.cpp
 * so nvcc never sees Eigen or heavyweight host headers (nvcc 11.x cannot
 * parse post-11.2 libstdc++ <functional>, and the split keeps that class
 * of problem structurally impossible).
 *
 * Kernel layout: one thread per sample, each rolling its perturbed control
 * sequence forward and accumulating stage + importance-sampling costs —
 * the "split rollout" arrangement of MPPI-Generic, the right shape for
 * small-state models like diff-drive (the state fits in registers; the
 * per-dimension-parallel fused variant pays off only for large states).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <cstddef>

#include "xmnav/mppi/cuda/wheeled_program.hpp"

namespace xmotion {

namespace {

__global__ void WheeledRolloutKernel(WheeledProgram prog,
                                     WheeledKernelConfig cfg,
                                     const float *__restrict__ u,
                                     const float *__restrict__ noise,
                                     float *__restrict__ costs) {
  const int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= cfg.num_samples) return;

  const float *eps = noise + static_cast<std::size_t>(k) * cfg.horizon * 2;
  float x[3] = {cfg.x0[0], cfg.x0[1], cfg.x0[2]};
  float cost = 0.0f;
  for (int t = 0; t < cfg.horizon; ++t) {
    float v[2];
#pragma unroll
    for (int j = 0; j < 2; ++j) {
      v[j] = u[t * 2 + j] + eps[t * 2 + j];
      v[j] = fminf(fmaxf(v[j], cfg.u_min[j]), cfg.u_max[j]);
    }
    float next[3];
    prog.Step(x, v, cfg.dt, next);
    x[0] = next[0];
    x[1] = next[1];
    x[2] = next[2];
    cost += prog.StageCost(x, v);
    // importance-sampling correction, same form as the CPU backend
    cost += cfg.gamma * (u[t * 2] * cfg.sigma_inv_sq[0] * eps[t * 2] +
                         u[t * 2 + 1] * cfg.sigma_inv_sq[1] * eps[t * 2 + 1]);
  }
  cost += prog.TerminalCost(x);
  costs[k] = cost;
}

}  // namespace

int LaunchWheeledRollouts(const WheeledProgram &prog,
                          const WheeledKernelConfig &cfg, const float *d_u,
                          const float *d_noise, float *d_costs) {
  const int block = 128;
  const int grid = (cfg.num_samples + block - 1) / block;
  WheeledRolloutKernel<<<grid, block>>>(prog, cfg, d_u, d_noise, d_costs);
  return static_cast<int>(cudaPeekAtLastError());
}

}  // namespace xmotion
