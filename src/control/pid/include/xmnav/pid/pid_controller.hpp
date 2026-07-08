/*
 * @file pid_controller.hpp
 * @brief Production single-channel PID controller.
 *
 * Replaces the float-based legacy implementation with the standard
 * production form:
 *  - derivative on MEASUREMENT (a setpoint step produces no derivative
 *    kick) with an optional first-order low-pass on the derivative
 *    (d_filter_tau; 0 disables — raw differentiation amplifies noise)
 *  - two anti-windup strategies: conditional integration (the integral
 *    is not advanced while the output saturates further in the error's
 *    direction) or back-calculation (integral bleeds toward the
 *    saturated output at back_calc_gain)
 *  - dt passed per Update() call (control loops with jitter), gains
 *    live-tunable between calls (integral state carries ki inside, so a
 *    gain change causes no output bump)
 *  - defensive boundary: a non-finite input or dt <= 0 leaves the state
 *    untouched and returns the last output (system-level degradation is
 *    the safety shield's job)
 *
 * Units are the caller's; kp/ki/kd follow the parallel form
 * u = kp e + integral(ki e dt) - kd d(y)/dt.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_PID_PID_CONTROLLER_HPP
#define XMNAV_PID_PID_CONTROLLER_HPP

#include <algorithm>
#include <cmath>
#include <limits>

namespace xmotion {

class PidController {
 public:
  enum class AntiWindup {
    kConditionalIntegration,
    kBackCalculation,
  };

  struct Config {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double u_min = -std::numeric_limits<double>::infinity();
    double u_max = std::numeric_limits<double>::infinity();
    // first-order low-pass time constant on the derivative term [s];
    // 0 differentiates raw
    double d_filter_tau = 0.0;
    AntiWindup anti_windup = AntiWindup::kConditionalIntegration;
    // integral bleed rate toward the saturated output [1/s]
    // (kBackCalculation only)
    double back_calc_gain = 1.0;
  };

  explicit PidController(const Config &config) : config_(config) {}

  double Update(double reference, double measurement, double dt) {
    if (!std::isfinite(reference) || !std::isfinite(measurement) ||
        !(dt > 0.0)) {
      return last_output_;  // defensive: state untouched
    }
    const double error = reference - measurement;

    // derivative on measurement, optionally low-passed
    double d_meas = 0.0;
    if (has_previous_) {
      d_meas = (measurement - previous_measurement_) / dt;
      if (config_.d_filter_tau > 0.0) {
        const double a = dt / (config_.d_filter_tau + dt);
        derivative_ += a * (d_meas - derivative_);
      } else {
        derivative_ = d_meas;
      }
    }
    previous_measurement_ = measurement;
    has_previous_ = true;

    const double candidate_integral = integral_ + config_.ki * error * dt;
    const double p = config_.kp * error;
    const double d = -config_.kd * derivative_;
    const double unsat = p + candidate_integral + d;
    const double sat = std::clamp(unsat, config_.u_min, config_.u_max);

    switch (config_.anti_windup) {
      case AntiWindup::kConditionalIntegration:
        // advance the integral unless the output is saturated AND the
        // error pushes further into the saturation
        if (sat == unsat || (unsat > config_.u_max) != (error > 0.0)) {
          integral_ = candidate_integral;
        }
        break;
      case AntiWindup::kBackCalculation:
        integral_ =
            candidate_integral + config_.back_calc_gain * (sat - unsat) * dt;
        break;
    }

    last_output_ = sat;
    return sat;
  }

  void Reset() {
    integral_ = 0.0;
    derivative_ = 0.0;
    previous_measurement_ = 0.0;
    has_previous_ = false;
    last_output_ = 0.0;
  }

  // live tuning between Update() calls; the integral state already
  // carries ki, so changing gains does not bump the output
  void SetGains(double kp, double ki, double kd) {
    config_.kp = kp;
    config_.ki = ki;
    config_.kd = kd;
  }

  const Config &config() const { return config_; }
  double integral() const { return integral_; }
  double derivative() const { return derivative_; }
  double last_output() const { return last_output_; }

 private:
  Config config_;
  double integral_ = 0.0;
  double derivative_ = 0.0;
  double previous_measurement_ = 0.0;
  bool has_previous_ = false;
  double last_output_ = 0.0;
};

}  // namespace xmotion

#endif  // XMNAV_PID_PID_CONTROLLER_HPP
