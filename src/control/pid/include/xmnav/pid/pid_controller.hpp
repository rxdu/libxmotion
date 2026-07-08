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
 *  - setpoint weighting (2-DOF form): the P term acts on
 *    setpoint_weight * r - y; 1 is the classic 1-DOF behavior, 0 removes
 *    the proportional jump on setpoint steps entirely
 *  - bumpless transfer: AlignOutput() seeds the integral so the first
 *    Update() after switching from manual (or another controller)
 *    continues from the current actuator output
 *  - observability: a non-empty Config::name exports
 *    control.pid.<name>.{output,error} gauges and a .saturated counter
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
#include <optional>
#include <string>

#include "xmbase/telemetry/telemetry.hpp"

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
    // P-term setpoint weight b in [0, 1]: P = kp (b r - y)
    double setpoint_weight = 1.0;
    // instance name for telemetry (control.pid.<name>.*); empty disables
    std::string name{};
  };

  explicit PidController(const Config &config) : config_(config) {
    if (!config_.name.empty()) {
      const std::string prefix = "control.pid." + config_.name;
      output_gauge_ = telemetry::GetGauge(prefix + ".output");
      error_gauge_ = telemetry::GetGauge(prefix + ".error");
      saturated_counter_ = telemetry::GetCounter(prefix + ".saturated");
    }
  }

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
    const double p =
        config_.kp * (config_.setpoint_weight * reference - measurement);
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
    if (output_gauge_) {
      output_gauge_->Set(sat);
      error_gauge_->Set(error);
      if (sat != unsat) saturated_counter_->Add();
    }
    return sat;
  }

  // Bumpless transfer: seed the internal state so the next Update() at
  // the same operating point continues from u_current (e.g. switching
  // from manual mode or handing over from another controller). The
  // derivative state restarts settled.
  void AlignOutput(double u_current, double reference, double measurement) {
    if (!std::isfinite(u_current) || !std::isfinite(reference) ||
        !std::isfinite(measurement)) {
      return;
    }
    previous_measurement_ = measurement;
    has_previous_ = true;
    derivative_ = 0.0;
    const double u = std::clamp(u_current, config_.u_min, config_.u_max);
    integral_ = u - config_.kp * (config_.setpoint_weight * reference -
                                  measurement);
    last_output_ = u;
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
  std::optional<telemetry::Gauge> output_gauge_;
  std::optional<telemetry::Gauge> error_gauge_;
  std::optional<telemetry::Counter> saturated_counter_;
};

}  // namespace xmotion

#endif  // XMNAV_PID_PID_CONTROLLER_HPP
