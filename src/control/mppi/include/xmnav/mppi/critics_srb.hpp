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
    const double dh = SrbQuadrupedModel::Position(x)(2) - height_ref;
    const Eigen::Vector3d body_z =
        SrbQuadrupedModel::Orientation(x) * Eigen::Vector3d::UnitZ();
    const double tilt = 1.0 - body_z(2);  // 0 when level
    const Eigen::Vector3d dv = SrbQuadrupedModel::Velocity(x) - velocity_ref;
    const Eigen::Vector3d w = SrbQuadrupedModel::AngularVelocity(x);
    return height_weight * dh * dh + tilt_weight * tilt +
           velocity_weight * dv.squaredNorm() +
           angular_rate_weight * w.squaredNorm();
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
      const Eigen::Vector3d f = u.segment<3>(3 * i);
      const double pull = std::max(0.0, -f(2));           // f_z >= 0
      const double slip =
          std::max(0.0, f.head<2>().norm() - mu * std::max(0.0, f(2)));
      cost += weight * (pull * pull + slip * slip);
    }
    return cost;
  }
  double TerminalCost(const State & /*x*/) const { return 0.0; }
};

}  // namespace xmotion

#endif  // XMNAV_MPPI_CRITICS_SRB_HPP
