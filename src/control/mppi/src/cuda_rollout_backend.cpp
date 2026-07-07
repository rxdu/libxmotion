/*
 * cuda_rollout_backend.cpp
 *
 * Host side of the CUDA rollout backends: buffer management (pinned host +
 * device pairs, grown once and reused — steady-state calls perform no
 * allocation), double<->float staging, program construction from the live
 * model/cost objects, and error translation. Compiled by the host
 * compiler; only the kernels (cuda_rollout_backend.cu) go through nvcc.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <cuda_runtime.h>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

#include "xmnav/mppi/cuda/cuda_rollout_backend.hpp"
#include "xmnav/mppi/cuda/kernels.hpp"

namespace xmotion {

namespace {

void Check(cudaError_t err, const char *what) {
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string("CUDA error in ") + what + ": " +
                             cudaGetErrorString(err));
  }
}

void Check(int err, const char *what) {
  Check(static_cast<cudaError_t>(err), what);
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

SrbProgram MakeSrbProgram(const SrbQuadrupedModel &model,
                          const SrbTrackingCost &tracking,
                          const FrictionConeCost &cone,
                          const QuadraticControlCost<13, 12> &reg,
                          int horizon) {
  if (horizon > SrbProgram::kMaxHorizon) {
    throw std::invalid_argument(
        "SrbProgram: horizon exceeds kMaxHorizon");
  }
  SrbProgram p{};
  p.mass = static_cast<float>(model.params().mass);
  p.gravity = static_cast<float>(model.params().gravity);
  for (int i = 0; i < 3; ++i) {
    p.inertia_diag[i] = static_cast<float>(model.params().inertia_diag(i));
  }
  for (int t = 0; t < horizon; ++t) {
    const auto &feet = model.FeetAt(t);
    for (int i = 0; i < SrbProgram::kNumFeet; ++i) {
      for (int c = 0; c < 3; ++c) {
        p.feet[12 * t + 3 * i + c] =
            static_cast<float>(feet[static_cast<std::size_t>(i)](c));
      }
      const bool stance = model.InStance(t, i);
      // one stance table serves the dynamics and the cone critic; a
      // schedule mismatch between them silently distorts costs on the
      // CPU too, so treat it as a configuration error here
      if (cone.InStance(t, i) != stance) {
        throw std::invalid_argument(
            "SrbProgram: friction-cone schedule disagrees with the model "
            "contact schedule");
      }
      p.stance[SrbProgram::kNumFeet * t + i] = stance ? 1 : 0;
    }
  }
  p.height_ref = static_cast<float>(tracking.height_ref);
  for (int i = 0; i < 3; ++i) {
    p.velocity_ref[i] = static_cast<float>(tracking.velocity_ref(i));
  }
  p.height_weight = static_cast<float>(tracking.height_weight);
  p.tilt_weight = static_cast<float>(tracking.tilt_weight);
  p.velocity_weight = static_cast<float>(tracking.velocity_weight);
  p.angular_rate_weight = static_cast<float>(tracking.angular_rate_weight);
  p.terminal_scale = static_cast<float>(tracking.terminal_scale);
  p.mu = static_cast<float>(cone.mu);
  p.cone_weight = static_cast<float>(cone.weight);
  for (int i = 0; i < 12; ++i) {
    for (int j = 0; j < 12; ++j) {
      if (i != j && std::abs(reg.R(i, j)) > 0.0) {
        throw std::invalid_argument(
            "SrbProgram: device control regularization supports diagonal R "
            "only");
      }
    }
    p.r_diag[i] = static_cast<float>(reg.R(i, i));
  }
  return p;
}

}  // namespace

bool CudaDeviceAvailable() {
  int count = 0;
  return cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
}

namespace cuda_detail {

struct DeviceWorkspace {
  float *h_u = nullptr;        // pinned, horizon*CD
  float *h_noise = nullptr;    // pinned, K*horizon*CD
  float *h_costs = nullptr;    // pinned, K (doubles as the weights staging)
  float *h_u_delta = nullptr;  // pinned, horizon*CD
  float *d_u = nullptr;
  float *d_noise = nullptr;
  float *d_costs = nullptr;
  float *d_weights = nullptr;
  float *d_u_delta = nullptr;
  void *d_prog = nullptr;  // device copy of the rollout program
  std::size_t u_capacity = 0;
  std::size_t noise_capacity = 0;
  std::size_t cost_capacity = 0;
  std::size_t prog_capacity = 0;

  ~DeviceWorkspace() {
    // best-effort teardown; errors here are not actionable
    cudaFreeHost(h_u);
    cudaFreeHost(h_noise);
    cudaFreeHost(h_costs);
    cudaFreeHost(h_u_delta);
    cudaFree(d_u);
    cudaFree(d_noise);
    cudaFree(d_costs);
    cudaFree(d_weights);
    cudaFree(d_u_delta);
    cudaFree(d_prog);
  }

  static void GrowPinned(float **p, std::size_t bytes) {
    Check(cudaFreeHost(*p), "cudaFreeHost");
    Check(cudaHostAlloc(reinterpret_cast<void **>(p), bytes,
                        cudaHostAllocDefault),
          "cudaHostAlloc");
  }
  static void GrowDevice(void **p, std::size_t bytes) {
    Check(cudaFree(*p), "cudaFree");
    Check(cudaMalloc(p, bytes), "cudaMalloc");
  }

  void EnsureCapacity(int num_samples, int horizon, int control_dim,
                      std::size_t prog_bytes) {
    const std::size_t u_n =
        static_cast<std::size_t>(horizon) * control_dim;
    const std::size_t noise_n = static_cast<std::size_t>(num_samples) * u_n;
    const std::size_t cost_n = static_cast<std::size_t>(num_samples);
    if (u_n > u_capacity) {
      GrowPinned(&h_u, u_n * sizeof(float));
      GrowPinned(&h_u_delta, u_n * sizeof(float));
      GrowDevice(reinterpret_cast<void **>(&d_u), u_n * sizeof(float));
      GrowDevice(reinterpret_cast<void **>(&d_u_delta),
                 u_n * sizeof(float));
      u_capacity = u_n;
    }
    if (noise_n > noise_capacity) {
      GrowPinned(&h_noise, noise_n * sizeof(float));
      GrowDevice(reinterpret_cast<void **>(&d_noise),
                 noise_n * sizeof(float));
      noise_capacity = noise_n;
    }
    if (cost_n > cost_capacity) {
      GrowPinned(&h_costs, cost_n * sizeof(float));
      GrowDevice(reinterpret_cast<void **>(&d_costs),
                 cost_n * sizeof(float));
      GrowDevice(reinterpret_cast<void **>(&d_weights),
                 cost_n * sizeof(float));
      cost_capacity = cost_n;
    }
    if (prog_bytes > prog_capacity) {
      GrowDevice(&d_prog, prog_bytes);
      prog_capacity = prog_bytes;
    }
  }
};

}  // namespace cuda_detail

namespace {

using cuda_detail::DeviceWorkspace;

// shared upload-evaluate-download path of the two upload backends
template <typename Prog, typename Backend>
void EvaluateUploadPath(DeviceWorkspace &ws, const Prog &prog,
                        const typename Backend::Params &params, double gamma,
                        const typename Backend::State &x0,
                        const typename Backend::ControlSequence &u,
                        const std::vector<typename Backend::ControlSequence>
                            &noise,
                        const typename Backend::Control &sigma_inv_sq,
                        Eigen::VectorXd &costs) {
  constexpr int SD = Backend::kStateDim;
  constexpr int CD = Backend::kControlDim;
  const int K = params.num_samples;
  const int T = params.horizon_steps;
  ws.EnsureCapacity(K, T, CD, sizeof(Prog));

  for (int t = 0; t < T; ++t) {
    for (int j = 0; j < CD; ++j) {
      ws.h_u[t * CD + j] = static_cast<float>(u(t, j));
    }
  }
  for (int k = 0; k < K; ++k) {
    const auto &eps = noise[static_cast<std::size_t>(k)];
    float *dst = ws.h_noise + static_cast<std::size_t>(k) * T * CD;
    for (int t = 0; t < T; ++t) {
      for (int j = 0; j < CD; ++j) {
        dst[t * CD + j] = static_cast<float>(eps(t, j));
      }
    }
  }

  RolloutConfig<SD, CD> cfg{};
  for (int i = 0; i < SD; ++i) cfg.x0[i] = static_cast<float>(x0(i));
  for (int j = 0; j < CD; ++j) {
    cfg.u_min[j] = static_cast<float>(params.u_min(j));
    cfg.u_max[j] = static_cast<float>(params.u_max(j));
    cfg.sigma_inv_sq[j] = static_cast<float>(sigma_inv_sq(j));
    cfg.sigma[j] = static_cast<float>(params.sigma(j));
  }
  cfg.dt = static_cast<float>(params.dt);
  cfg.gamma = static_cast<float>(gamma);
  cfg.num_samples = K;
  cfg.horizon = T;

  Check(cudaMemcpy(ws.d_prog, &prog, sizeof(Prog), cudaMemcpyHostToDevice),
        "upload program");
  Check(cudaMemcpy(ws.d_u, ws.h_u,
                   static_cast<std::size_t>(T) * CD * sizeof(float),
                   cudaMemcpyHostToDevice),
        "upload u");
  Check(cudaMemcpy(ws.d_noise, ws.h_noise,
                   static_cast<std::size_t>(K) * T * CD * sizeof(float),
                   cudaMemcpyHostToDevice),
        "upload noise");
  Check(LaunchRollouts(static_cast<const Prog *>(ws.d_prog), cfg, ws.d_u,
                       ws.d_noise, ws.d_costs),
        "kernel launch");
  Check(cudaMemcpy(ws.h_costs, ws.d_costs,
                   static_cast<std::size_t>(K) * sizeof(float),
                   cudaMemcpyDeviceToHost),
        "download costs");
  for (int k = 0; k < K; ++k) {
    costs(k) = static_cast<double>(ws.h_costs[k]);
  }
}

void RequireDevice(const char *who) {
  if (!CudaDeviceAvailable()) {
    throw std::runtime_error(std::string(who) + ": no usable CUDA device");
  }
}

}  // namespace

// --- CudaWheeledRolloutBackend ---

CudaWheeledRolloutBackend::CudaWheeledRolloutBackend()
    : ws_(new DeviceWorkspace) {
  RequireDevice("CudaWheeledRolloutBackend");
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
  // rebuilt from the live cost object on every call, so critic mutations
  // between Plan() calls behave exactly as on the CPU
  const WheeledProgram prog = MakeWheeledProgram(
      cost.template critic<0>(), cost.template critic<1>());
  EvaluateUploadPath<WheeledProgram, CudaWheeledRolloutBackend>(
      *ws_, prog, params, gamma, x0, u, noise, sigma_inv_sq, costs);
}

// --- CudaSrbRolloutBackend ---

CudaSrbRolloutBackend::CudaSrbRolloutBackend() : ws_(new DeviceWorkspace) {
  RequireDevice("CudaSrbRolloutBackend");
}
CudaSrbRolloutBackend::~CudaSrbRolloutBackend() = default;
CudaSrbRolloutBackend::CudaSrbRolloutBackend(
    CudaSrbRolloutBackend &&) noexcept = default;
CudaSrbRolloutBackend &CudaSrbRolloutBackend::operator=(
    CudaSrbRolloutBackend &&) noexcept = default;

void CudaSrbRolloutBackend::Evaluate(
    const Model &model, const Cost &cost, const Params &params, double gamma,
    const State &x0, const ControlSequence &u,
    const std::vector<ControlSequence> &noise, const Control &sigma_inv_sq,
    Eigen::VectorXd &costs) {
  const SrbProgram prog = MakeSrbProgram(
      model, cost.template critic<0>(), cost.template critic<1>(),
      cost.template critic<2>(), params.horizon_steps);
  EvaluateUploadPath<SrbProgram, CudaSrbRolloutBackend>(
      *ws_, prog, params, gamma, x0, u, noise, sigma_inv_sq, costs);
}

// --- CudaWheeledSamplingBackend ---

CudaWheeledSamplingBackend::CudaWheeledSamplingBackend(std::uint64_t seed)
    : ws_(new DeviceWorkspace), seed_(seed) {
  RequireDevice("CudaWheeledSamplingBackend");
}
CudaWheeledSamplingBackend::~CudaWheeledSamplingBackend() = default;
CudaWheeledSamplingBackend::CudaWheeledSamplingBackend(
    CudaWheeledSamplingBackend &&) noexcept = default;
CudaWheeledSamplingBackend &CudaWheeledSamplingBackend::operator=(
    CudaWheeledSamplingBackend &&) noexcept = default;

void CudaWheeledSamplingBackend::Evaluate(
    const Model & /*model*/, const Cost &cost, const Params &params,
    double gamma, const State &x0, const ControlSequence &u,
    const std::vector<ControlSequence> & /*noise: drawn on-device*/,
    const Control &sigma_inv_sq, Eigen::VectorXd &costs) {
  constexpr int CD = kControlDim;
  const int K = params.num_samples;
  const int T = params.horizon_steps;
  ws_->EnsureCapacity(K, T, CD, sizeof(WheeledProgram));

  for (int t = 0; t < T; ++t) {
    for (int j = 0; j < CD; ++j) {
      ws_->h_u[t * CD + j] = static_cast<float>(u(t, j));
    }
  }
  RolloutConfig<kStateDim, CD> cfg{};
  for (int i = 0; i < kStateDim; ++i) {
    cfg.x0[i] = static_cast<float>(x0(i));
  }
  for (int j = 0; j < CD; ++j) {
    cfg.u_min[j] = static_cast<float>(params.u_min(j));
    cfg.u_max[j] = static_cast<float>(params.u_max(j));
    cfg.sigma_inv_sq[j] = static_cast<float>(sigma_inv_sq(j));
    cfg.sigma[j] = static_cast<float>(params.sigma(j));
  }
  cfg.dt = static_cast<float>(params.dt);
  cfg.gamma = static_cast<float>(gamma);
  cfg.num_samples = K;
  cfg.horizon = T;

  const WheeledProgram prog = MakeWheeledProgram(
      cost.template critic<0>(), cost.template critic<1>());
  Check(cudaMemcpy(ws_->d_prog, &prog, sizeof(prog),
                   cudaMemcpyHostToDevice),
        "upload program");
  Check(cudaMemcpy(ws_->d_u, ws_->h_u,
                   static_cast<std::size_t>(T) * CD * sizeof(float),
                   cudaMemcpyHostToDevice),
        "upload u");
  Check(LaunchSampledRollouts(static_cast<const WheeledProgram *>(ws_->d_prog),
                              cfg, seed_, stream_offset_, ws_->d_u,
                              ws_->d_noise, ws_->d_costs),
        "kernel launch");
  // advance every plan so streams never overlap (upper bound on the
  // uniforms one normal draw can consume)
  stream_offset_ += static_cast<unsigned long long>(T) * CD * 4;
  last_num_samples_ = K;
  last_horizon_ = T;

  Check(cudaMemcpy(ws_->h_costs, ws_->d_costs,
                   static_cast<std::size_t>(K) * sizeof(float),
                   cudaMemcpyDeviceToHost),
        "download costs");
  for (int k = 0; k < K; ++k) {
    costs(k) = static_cast<double>(ws_->h_costs[k]);
  }
}

void CudaWheeledSamplingBackend::ApplyWeightedUpdate(
    const Eigen::VectorXd &weights, ControlSequence &u_delta) {
  constexpr int CD = kControlDim;
  const int K = last_num_samples_;
  const int T = last_horizon_;
  if (K == 0 || weights.size() != K) {
    throw std::logic_error(
        "ApplyWeightedUpdate: weights do not match the last Evaluate()");
  }
  for (int k = 0; k < K; ++k) {
    ws_->h_costs[k] = static_cast<float>(weights(k));  // staging reuse
  }
  Check(cudaMemcpy(ws_->d_weights, ws_->h_costs,
                   static_cast<std::size_t>(K) * sizeof(float),
                   cudaMemcpyHostToDevice),
        "upload weights");
  Check(LaunchWeightedUpdate(ws_->d_noise, ws_->d_weights, K, T * CD,
                             ws_->d_u_delta),
        "weighted-update launch");
  Check(cudaMemcpy(ws_->h_u_delta, ws_->d_u_delta,
                   static_cast<std::size_t>(T) * CD * sizeof(float),
                   cudaMemcpyDeviceToHost),
        "download u_delta");
  u_delta.resize(T, CD);
  for (int t = 0; t < T; ++t) {
    for (int j = 0; j < CD; ++j) {
      u_delta(t, j) = static_cast<double>(ws_->h_u_delta[t * CD + j]);
    }
  }
}

void CudaWheeledSamplingBackend::DownloadNoiseSample(
    int k, ControlSequence &eps) const {
  constexpr int CD = kControlDim;
  const int T = last_horizon_;
  if (k < 0 || k >= last_num_samples_) {
    throw std::out_of_range("DownloadNoiseSample: sample index");
  }
  eps.resize(T, CD);
  // small transfer (T*CD floats), staged through h_u_delta scratch
  Check(cudaMemcpy(ws_->h_u_delta,
                   ws_->d_noise + static_cast<std::size_t>(k) * T * CD,
                   static_cast<std::size_t>(T) * CD * sizeof(float),
                   cudaMemcpyDeviceToHost),
        "download noise sample");
  for (int t = 0; t < T; ++t) {
    for (int j = 0; j < CD; ++j) {
      eps(t, j) = static_cast<double>(ws_->h_u_delta[t * CD + j]);
    }
  }
}

}  // namespace xmotion
