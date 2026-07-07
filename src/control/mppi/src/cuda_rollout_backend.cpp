/*
 * cuda_rollout_backend.cpp
 *
 * Host side of the CUDA rollout backend: buffer management (pinned host +
 * device pairs, grown once and reused — steady-state Evaluate() performs
 * no allocation), double<->float staging, program construction from the
 * live cost object, and error translation. Compiled by the host compiler;
 * only the kernel launch (cuda_rollout_backend.cu) goes through nvcc.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <cuda_runtime.h>

#include <cstddef>
#include <stdexcept>
#include <string>

#include "xmnav/mppi/cuda/cuda_rollout_backend.hpp"
#include "xmnav/mppi/cuda/wheeled_program.hpp"

namespace xmotion {

namespace {

void Check(cudaError_t err, const char *what) {
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string("CUDA error in ") + what + ": " +
                             cudaGetErrorString(err));
  }
}

WheeledProgram MakeWheeledProgram(const Se2GoalCost &goal_cost,
                                  const CircularObstacleCost &obstacle_cost) {
  if (static_cast<int>(obstacle_cost.obstacles.size()) >
      WheeledProgram::kMaxObstacles) {
    throw std::invalid_argument(
        "WheeledProgram: obstacle count exceeds kMaxObstacles");
  }
  WheeledProgram p{};
  for (int i = 0; i < 3; ++i) {
    p.goal[i] = static_cast<float>(goal_cost.goal(i));
  }
  p.position_weight = static_cast<float>(goal_cost.position_weight);
  p.heading_weight = static_cast<float>(goal_cost.heading_weight);
  p.terminal_scale = static_cast<float>(goal_cost.terminal_scale);
  p.num_obstacles = static_cast<int>(obstacle_cost.obstacles.size());
  for (int i = 0; i < p.num_obstacles; ++i) {
    const auto &ob = obstacle_cost.obstacles[static_cast<std::size_t>(i)];
    p.obstacles[3 * i] = static_cast<float>(ob.center(0));
    p.obstacles[3 * i + 1] = static_cast<float>(ob.center(1));
    p.obstacles[3 * i + 2] = static_cast<float>(ob.radius);
  }
  p.obstacle_weight = static_cast<float>(obstacle_cost.weight);
  p.obstacle_margin = static_cast<float>(obstacle_cost.margin);
  return p;
}

}  // namespace

bool CudaDeviceAvailable() {
  int count = 0;
  return cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
}

struct CudaWheeledRolloutBackend::Impl {
  float *h_u = nullptr;      // pinned, horizon*2
  float *h_noise = nullptr;  // pinned, K*horizon*2
  float *h_costs = nullptr;  // pinned, K
  float *d_u = nullptr;
  float *d_noise = nullptr;
  float *d_costs = nullptr;
  std::size_t u_capacity = 0;      // floats
  std::size_t noise_capacity = 0;  // floats
  std::size_t cost_capacity = 0;   // floats

  ~Impl() {
    // best-effort teardown; errors here are not actionable
    cudaFreeHost(h_u);
    cudaFreeHost(h_noise);
    cudaFreeHost(h_costs);
    cudaFree(d_u);
    cudaFree(d_noise);
    cudaFree(d_costs);
  }

  // grow a pinned-host/device buffer pair together (one capacity counter)
  static void GrowPair(float **host, float **device, std::size_t *capacity,
                       std::size_t needed) {
    if (needed <= *capacity) return;
    Check(cudaFreeHost(*host), "cudaFreeHost");
    Check(cudaFree(*device), "cudaFree");
    Check(cudaHostAlloc(reinterpret_cast<void **>(host),
                        needed * sizeof(float), cudaHostAllocDefault),
          "cudaHostAlloc");
    Check(cudaMalloc(reinterpret_cast<void **>(device),
                     needed * sizeof(float)),
          "cudaMalloc");
    *capacity = needed;
  }

  void EnsureCapacity(int num_samples, int horizon) {
    const std::size_t u_n = static_cast<std::size_t>(horizon) * 2;
    const std::size_t noise_n = static_cast<std::size_t>(num_samples) * u_n;
    const std::size_t cost_n = static_cast<std::size_t>(num_samples);
    GrowPair(&h_u, &d_u, &u_capacity, u_n);
    GrowPair(&h_noise, &d_noise, &noise_capacity, noise_n);
    GrowPair(&h_costs, &d_costs, &cost_capacity, cost_n);
  }
};

CudaWheeledRolloutBackend::CudaWheeledRolloutBackend() : impl_(new Impl) {
  if (!CudaDeviceAvailable()) {
    throw std::runtime_error(
        "CudaWheeledRolloutBackend: no usable CUDA device");
  }
}

CudaWheeledRolloutBackend::~CudaWheeledRolloutBackend() = default;
CudaWheeledRolloutBackend::CudaWheeledRolloutBackend(
    CudaWheeledRolloutBackend &&) noexcept = default;
CudaWheeledRolloutBackend &CudaWheeledRolloutBackend::operator=(
    CudaWheeledRolloutBackend &&) noexcept = default;

void CudaWheeledRolloutBackend::Evaluate(
    const Model & /*model: stateless, dynamics live in the program*/,
    const Cost &cost, const Params &params, double gamma, const State &x0,
    const ControlSequence &u, const std::vector<ControlSequence> &noise,
    const Control &sigma_inv_sq, Eigen::VectorXd &costs) {
  const int K = params.num_samples;
  const int T = params.horizon_steps;
  impl_->EnsureCapacity(K, T);

  // narrow to the float staging buffers (row-major [t][dim] flattening)
  for (int t = 0; t < T; ++t) {
    impl_->h_u[t * 2] = static_cast<float>(u(t, 0));
    impl_->h_u[t * 2 + 1] = static_cast<float>(u(t, 1));
  }
  for (int k = 0; k < K; ++k) {
    const ControlSequence &eps = noise[static_cast<std::size_t>(k)];
    float *dst = impl_->h_noise + static_cast<std::size_t>(k) * T * 2;
    for (int t = 0; t < T; ++t) {
      dst[t * 2] = static_cast<float>(eps(t, 0));
      dst[t * 2 + 1] = static_cast<float>(eps(t, 1));
    }
  }

  WheeledKernelConfig cfg{};
  for (int i = 0; i < 3; ++i) cfg.x0[i] = static_cast<float>(x0(i));
  for (int j = 0; j < 2; ++j) {
    cfg.u_min[j] = static_cast<float>(params.u_min(j));
    cfg.u_max[j] = static_cast<float>(params.u_max(j));
    cfg.sigma_inv_sq[j] = static_cast<float>(sigma_inv_sq(j));
  }
  cfg.dt = static_cast<float>(params.dt);
  cfg.gamma = static_cast<float>(gamma);
  cfg.num_samples = K;
  cfg.horizon = T;

  // rebuilt from the live cost object on every call, so critic mutations
  // between Plan() calls behave exactly as on the CPU
  const WheeledProgram prog = MakeWheeledProgram(
      cost.template critic<0>(), cost.template critic<1>());

  Check(cudaMemcpy(impl_->d_u, impl_->h_u,
                   static_cast<std::size_t>(T) * 2 * sizeof(float),
                   cudaMemcpyHostToDevice),
        "upload u");
  Check(cudaMemcpy(impl_->d_noise, impl_->h_noise,
                   static_cast<std::size_t>(K) * T * 2 * sizeof(float),
                   cudaMemcpyHostToDevice),
        "upload noise");
  Check(static_cast<cudaError_t>(LaunchWheeledRollouts(
            prog, cfg, impl_->d_u, impl_->d_noise, impl_->d_costs)),
        "kernel launch");
  Check(cudaMemcpy(impl_->h_costs, impl_->d_costs,
                   static_cast<std::size_t>(K) * sizeof(float),
                   cudaMemcpyDeviceToHost),
        "download costs");

  for (int k = 0; k < K; ++k) {
    costs(k) = static_cast<double>(impl_->h_costs[k]);
  }
}

}  // namespace xmotion
