/*
 * @file sim_loop.hpp
 * @brief Deterministic fixed-step simulation loop for algorithm development.
 *
 * The plant reuses the controller Model concept (Step(x, u, t, dt) with
 * compile-time dims) — the same model types serve as rollout models and as
 * truth plants, and the plant may deliberately differ from the model an
 * algorithm plans with (model-mismatch studies). Headless by design: this
 * header has no visualization dependencies; the viz-gated SimViewer2D
 * attaches through the observer callback.
 *
 * Determinism: fixed step, seeded noise, no wall-clock anywhere.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_SIM_SIM_LOOP_HPP
#define XMNAV_SIM_SIM_LOOP_HPP

#include <cstdint>
#include <functional>
#include <random>

#include <eigen3/Eigen/Dense>

#include "xmnav/sim/sim_log.hpp"

namespace xmotion {

template <typename Plant>
class SimLoop {
 public:
  static constexpr int kStateDim = Plant::kStateDim;
  static constexpr int kControlDim = Plant::kControlDim;

  using State = Eigen::Matrix<double, kStateDim, 1>;
  using Control = Eigen::Matrix<double, kControlDim, 1>;

  struct Config {
    double dt = 0.02;
    int steps = 500;
    std::uint64_t seed = 0;
    // per-channel std-dev of additive noise (zero = off)
    State process_noise = State::Zero();
    State measurement_noise = State::Zero();
  };

  // the algorithm under test: receives the (noisy) measured state and the
  // step index, returns the control to execute
  using Agent = std::function<Control(const State &, int)>;
  // observer: called after every executed step with (true state, control,
  // step index); the visual viewer attaches here, tests can assert here
  using Observer = std::function<void(const State &, const Control &, int)>;

  SimLoop(Plant plant, const Config &config)
      : plant_(std::move(plant)), config_(config), rng_(config.seed) {}

  Plant &plant() { return plant_; }
  const Config &config() const { return config_; }

  // run the scenario; returns the final true state. The log (optional)
  // records the true state, measurement, and control per step.
  State Run(const State &x0, const Agent &agent, SimLog *log = nullptr,
            const Observer &observer = {}) {
    return RunFrom(0, config_.steps, x0, agent, log, observer);
  }

  // run a segment: ticks t0..t0+steps-1 from state x0. Agent, plant, and
  // logged times all see the GLOBAL tick index, so time-dependent behavior
  // matches a full run. Resume contract (rewindability): the observer fires
  // at a clean RNG boundary — all of tick t's noise draws are complete when
  // it is called — so a COPY of this loop taken in the observer at tick k-1
  // carries the RNG mid-stream, and RunFrom(k, ...) on that copy from the
  // tick-(k-1) logged state reproduces the original run's tail bitwise.
  State RunFrom(int t0, int steps, const State &x0, const Agent &agent,
                SimLog *log = nullptr, const Observer &observer = {}) {
    if (log != nullptr) {
      log->Reset(steps, kStateDim, kControlDim);
    }
    State x = x0;
    for (int i = 0; i < steps; ++i) {
      const int t = t0 + i;
      const State z = x + Noise(config_.measurement_noise);
      const Control u = agent(z, t);
      x = plant_.Step(x, u, t, config_.dt);
      x += Noise(config_.process_noise);
      if (log != nullptr) {
        log->Append(t * config_.dt, x, z, u);
      }
      if (observer) {
        observer(x, u, t);
      }
    }
    return x;
  }

 private:
  State Noise(const State &sigma) {
    if (sigma.isZero()) return State::Zero();
    State n;
    for (int i = 0; i < kStateDim; ++i) {
      n(i) = sigma(i) * unit_normal_(rng_);
    }
    return n;
  }

  Plant plant_;
  Config config_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> unit_normal_{0.0, 1.0};
};

}  // namespace xmotion

#endif  // XMNAV_SIM_SIM_LOOP_HPP
