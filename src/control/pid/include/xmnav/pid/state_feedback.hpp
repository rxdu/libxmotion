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
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_PID_STATE_FEEDBACK_HPP
#define XMNAV_PID_STATE_FEEDBACK_HPP

#include <limits>

#include <eigen3/Eigen/Dense>

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
  };

  explicit StateFeedbackController(const Config &config)
      : config_(config) {}

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
    if constexpr (IntegralDim > 0) {
      // conditional integration: commit only while unsaturated
      if ((sat.array() == unsat.array()).all()) {
        z_ = z_candidate;
      }
    }
    last_output_ = sat;
    return sat;
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
};

}  // namespace xmotion

#endif  // XMNAV_PID_STATE_FEEDBACK_HPP
