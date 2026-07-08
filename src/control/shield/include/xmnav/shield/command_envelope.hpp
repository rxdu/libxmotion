/*
 * @file command_envelope.hpp
 * @brief Per-channel box + rate limits on controller commands.
 *
 * The model-free, always-on first layer of the safety shield: absolute
 * box clamps plus a slew-rate limit relative to the last *issued* command
 * (not the last requested one, so a runaway request cannot drag the
 * reference). Scenario S1 of docs/control/safety_shield.md.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_SHIELD_COMMAND_ENVELOPE_HPP
#define XMNAV_SHIELD_COMMAND_ENVELOPE_HPP

#include <limits>

#include <eigen3/Eigen/Dense>

namespace xmotion {

template <int ControlDim>
struct CommandEnvelope {
  using Control = Eigen::Matrix<double, ControlDim, 1>;

  // absolute per-channel bounds
  Control u_min =
      Control::Constant(-std::numeric_limits<double>::infinity());
  Control u_max = Control::Constant(std::numeric_limits<double>::infinity());
  // per-channel |du/dt| cap (infinity disables)
  Control rate_limit =
      Control::Constant(std::numeric_limits<double>::infinity());

  // box first, then slew toward the box-clamped target from u_prev;
  // u_prev is the last issued command and must be finite (the facade
  // guarantees it). Sets *clamped when any limit engaged.
  Control Apply(const Control &u, const Control &u_prev, double dt,
                bool *clamped = nullptr) const {
    Control out = u.cwiseMax(u_min).cwiseMin(u_max);
    for (int j = 0; j < ControlDim; ++j) {
      const double max_step = rate_limit(j) * dt;
      const double step = out(j) - u_prev(j);
      if (step > max_step) {
        out(j) = u_prev(j) + max_step;
      } else if (step < -max_step) {
        out(j) = u_prev(j) - max_step;
      }
    }
    if (clamped != nullptr) {
      *clamped = (out.array() != u.array()).any();
    }
    return out;
  }
};

}  // namespace xmotion

#endif  // XMNAV_SHIELD_COMMAND_ENVELOPE_HPP
