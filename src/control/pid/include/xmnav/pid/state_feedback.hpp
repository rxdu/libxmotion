/*
 * @file state_feedback.hpp
 * @brief Linear state-feedback controller with optional integral action.
 *
 * The multi-dimensional companion of PidController:
 *
 *   u = u_ff + K (x_ref - x) + Ki z,   z' = C (x_ref - x)
 *
 * K is the full state-feedback gain (design it with SolveDlqr from
 * dlqr.hpp, pole placement, or by hand); the optional integral
 * augmentation (IntegralDim > 0) integrates the tracked outputs
 * y = C x and removes steady-state offset under constant disturbances
 * (the LQI structure). Anti-windup: the integrator state is clamped
 * per channel (integrator_limit) and frozen while the output saturates
 * (conditional integration; with matrix coupling, per-channel
 * attribution is ambiguous, so saturation freezes the whole
 * integrator — conservative and simple).
 *
 * Same boundary contract as PidController: non-finite input or
 * dt <= 0 leaves the state untouched and returns the last output.
 * AlignOutput() provides bumpless transfer (IntegralDim > 0: the
 * integrator absorbs the current command in least squares; without an
 * integrator the law is static and there is no state to align). A
 * non-empty Config::name exports control.state_feedback.<name>.
 * {output_norm,error_norm} gauges and a .saturated counter.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_PID_STATE_FEEDBACK_HPP
#define XMNAV_PID_STATE_FEEDBACK_HPP

#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include <eigen3/Eigen/Dense>

#include "xmbase/telemetry/telemetry.hpp"

namespace xmotion {

template <int StateDim, int ControlDim, int IntegralDim = 0>
class StateFeedbackController {
 public:
  static constexpr int kStateDim = StateDim;
  static constexpr int kControlDim = ControlDim;
  static constexpr int kIntegralDim = IntegralDim;

  using State = Eigen::Matrix<double, StateDim, 1>;
  using Control = Eigen::Matrix<double, ControlDim, 1>;
  using Integral = Eigen::Matrix<double, IntegralDim, 1>;

  struct Config {
    // state-feedback gain on the state error
    Eigen::Matrix<double, ControlDim, StateDim> K =
        Eigen::Matrix<double, ControlDim, StateDim>::Zero();
    // integral gain and tracked-output selector (IntegralDim > 0)
    Eigen::Matrix<double, ControlDim, IntegralDim> Ki =
        Eigen::Matrix<double, ControlDim, IntegralDim>::Zero();
    Eigen::Matrix<double, IntegralDim, StateDim> C =
        Eigen::Matrix<double, IntegralDim, StateDim>::Zero();
    // feedforward / operating-point command
    Control u_ff = Control::Zero();
    // actuator box
    Control u_min =
        Control::Constant(-std::numeric_limits<double>::infinity());
    Control u_max =
        Control::Constant(std::numeric_limits<double>::infinity());
    // per-channel |z| clamp on the integrator state
    double integrator_limit = std::numeric_limits<double>::infinity();
    // instance name for telemetry (control.state_feedback.<name>.*);
    // empty disables
    std::string name{};
  };

  explicit StateFeedbackController(const Config &config)
      : config_(config) {
    if (!config_.name.empty()) {
      const std::string prefix = "control.state_feedback." + config_.name;
      output_gauge_ = telemetry::GetGauge(prefix + ".output_norm");
      error_gauge_ = telemetry::GetGauge(prefix + ".error_norm");
      saturated_counter_ = telemetry::GetCounter(prefix + ".saturated");
    }
  }

  Control Update(const State &x_ref, const State &x, double dt) {
    if (!x_ref.allFinite() || !x.allFinite() || !(dt > 0.0)) {
      return last_output_;  // defensive: state untouched
    }
    const State e = x_ref - x;
    Control unsat = config_.u_ff + config_.K * e;
    Integral z_candidate = z_;
    if constexpr (IntegralDim > 0) {
      z_candidate = (z_ + config_.C * e * dt)
                        .cwiseMax(-config_.integrator_limit)
                        .cwiseMin(config_.integrator_limit);
      unsat += config_.Ki * z_candidate;
    }
    const Control sat =
        unsat.cwiseMax(config_.u_min).cwiseMin(config_.u_max);
    const bool saturated = !(sat.array() == unsat.array()).all();
    if constexpr (IntegralDim > 0) {
      // conditional integration: commit only while unsaturated
      if (!saturated) {
        z_ = z_candidate;
      }
    }
    last_output_ = sat;
    if (output_gauge_) {
      output_gauge_->Set(sat.norm());
      error_gauge_->Set(e.norm());
      if (saturated) saturated_counter_->Add();
    }
    return sat;
  }

  // Bumpless transfer: seed the integrator so the next Update() at the
  // same operating point continues from u_current. With IntegralDim == 0
  // the law is static — only the last-output record is aligned.
  void AlignOutput(const Control &u_current, const State &x_ref,
                   const State &x) {
    if (!u_current.allFinite() || !x_ref.allFinite() || !x.allFinite()) {
      return;
    }
    const Control u =
        u_current.cwiseMax(config_.u_min).cwiseMin(config_.u_max);
    if constexpr (IntegralDim > 0) {
      // least-squares: Ki z = u - u_ff - K (x_ref - x)
      const Control rhs = u - config_.u_ff - config_.K * (x_ref - x);
      z_ = config_.Ki.completeOrthogonalDecomposition()
               .solve(rhs)
               .cwiseMax(-config_.integrator_limit)
               .cwiseMin(config_.integrator_limit);
    }
    last_output_ = u;
  }

  void Reset() {
    z_.setZero();
    last_output_.setZero();
  }

  const Config &config() const { return config_; }
  Config &config() { return config_; }
  const Integral &integrator() const { return z_; }
  const Control &last_output() const { return last_output_; }

 private:
  Config config_;
  Integral z_ = Integral::Zero();
  Control last_output_ = Control::Zero();
  std::optional<telemetry::Gauge> output_gauge_;
  std::optional<telemetry::Gauge> error_gauge_;
  std::optional<telemetry::Counter> saturated_counter_;
};

}  // namespace xmotion

#endif  // XMNAV_PID_STATE_FEEDBACK_HPP
