/*
 * @file critics_srb.hpp
 * @brief Cost critics for the single-rigid-body quadruped model.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MPPI_CRITICS_SRB_HPP
#define XMNAV_MPPI_CRITICS_SRB_HPP

#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/Dense>

// raw-span cores shared with the CUDA rollout backend
#include "xmnav/mppi/critic_core.hpp"
#include "xmnav/mppi/models/srb_quadruped.hpp"

namespace xmotion {

// Trunk tracking: height, level attitude (tilt of the body z axis), world
// velocity reference, angular-rate damping.
struct SrbTrackingCost {
  using State = SrbQuadrupedModel::State;
  using Control = SrbQuadrupedModel::Control;

  double height_ref = 0.28;
  Eigen::Vector3d velocity_ref = Eigen::Vector3d::Zero();

  double height_weight = 1500.0;
  double tilt_weight = 400.0;
  double velocity_weight = 60.0;
  // over-damping angular rate forbids the corrective rotations disturbance
  // absorption needs — keep this small (parameter sweep: 2 stable, 12 not)
  double angular_rate_weight = 2.0;
  double terminal_scale = 10.0;

  double StageCost(const State &x, const Control & /*u*/, int /*t*/) const {
    return critic_core::SrbTrackingStage(x.data(), height_ref,
                                         velocity_ref.data(), height_weight,
                                         tilt_weight, velocity_weight,
                                         angular_rate_weight);
  }
  double TerminalCost(const State &x) const {
    return terminal_scale * StageCost(x, Control::Zero(), 0);
  }
};

// Friction-cone soft constraint on stance-foot forces: f_z >= 0 (unilateral
// contact) and |f_xy| <= mu * f_z. Penalties are soft — the model already
// zero-masks swing feet, and a downstream leg controller enforces hard
// limits (see the technical note on constraint handling).
struct FrictionConeCost {
  using State = SrbQuadrupedModel::State;
  using Control = SrbQuadrupedModel::Control;

  SrbQuadrupedModel::ContactSchedule schedule;
  double mu = 0.6;
  double weight = 10.0;

  bool InStance(int t, int foot) const {
    if (schedule.empty()) return true;
    const std::size_t idx = std::min(static_cast<std::size_t>(t < 0 ? 0 : t),
                                     schedule.size() - 1);
    return schedule[idx][static_cast<std::size_t>(foot)];
  }

  double StageCost(const State & /*x*/, const Control &u, int t) const {
    double cost = 0.0;
    for (int i = 0; i < SrbQuadrupedModel::kNumFeet; ++i) {
      if (!InStance(t, i)) continue;
      cost += critic_core::FrictionConePenalty(u.data() + 3 * i, mu, weight);
    }
    return cost;
  }
  double TerminalCost(const State & /*x*/) const { return 0.0; }
};

}  // namespace xmotion

#endif  // XMNAV_MPPI_CRITICS_SRB_HPP
