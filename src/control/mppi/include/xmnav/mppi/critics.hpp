/*
 * @file critics.hpp
 * @brief Composable cost terms ("critics") for MPPI.
 *
 * The cost seam follows Nav2's production pattern: the total cost is a sum
 * of independent critics, each providing StageCost(state, control, t) and
 * TerminalCost(state). Compose is a compile-time sum, so the rollout loop
 * inlines everything.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MPPI_CRITICS_HPP
#define XMNAV_MPPI_CRITICS_HPP

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

#include <eigen3/Eigen/Dense>

namespace xmotion {

// (x - x_ref)^T Q (x - x_ref) per stage, scaled terminal weight at the end.
template <int StateDim, int ControlDim>
struct QuadraticStateCost {
  using State = Eigen::Matrix<double, StateDim, 1>;
  using Control = Eigen::Matrix<double, ControlDim, 1>;

  State x_ref = State::Zero();
  Eigen::Matrix<double, StateDim, StateDim> Q =
      Eigen::Matrix<double, StateDim, StateDim>::Identity();
  // terminal cost matrix; for tracking problems, setting this to the
  // Riccati solution makes the truncated horizon behave like an infinite one
  Eigen::Matrix<double, StateDim, StateDim> Q_terminal =
      Eigen::Matrix<double, StateDim, StateDim>::Identity() * 10.0;

  double StageCost(const State &x, const Control & /*u*/, int /*t*/) const {
    const State e = x - x_ref;
    return e.dot(Q * e);
  }
  double TerminalCost(const State &x) const {
    const State e = x - x_ref;
    return e.dot(Q_terminal * e);
  }
};

// u^T R u — explicit control effort (distinct from the derivation's
// importance-sampling KL term, which the controller adds itself).
template <int StateDim, int ControlDim>
struct QuadraticControlCost {
  using State = Eigen::Matrix<double, StateDim, 1>;
  using Control = Eigen::Matrix<double, ControlDim, 1>;

  Eigen::Matrix<double, ControlDim, ControlDim> R =
      Eigen::Matrix<double, ControlDim, ControlDim>::Identity();

  double StageCost(const State & /*x*/, const Control &u, int /*t*/) const {
    return u.dot(R * u);
  }
  double TerminalCost(const State & /*x*/) const { return 0.0; }
};

// SE(2) goal cost for planar robots: quadratic position error + heading via
// (1 - cos) so angle wrap is handled without branches.
struct Se2GoalCost {
  using State = Eigen::Matrix<double, 3, 1>;  // x, y, theta
  using Control = Eigen::Matrix<double, 2, 1>;

  State goal = State::Zero();
  double position_weight = 5.0;
  double heading_weight = 1.0;
  double terminal_scale = 20.0;

  double StageCost(const State &x, const Control & /*u*/, int /*t*/) const {
    const double dx = x(0) - goal(0);
    const double dy = x(1) - goal(1);
    return position_weight * (dx * dx + dy * dy) +
           heading_weight * (1.0 - std::cos(x(2) - goal(2)));
  }
  double TerminalCost(const State &x) const {
    return terminal_scale * StageCost(x, Control::Zero(), 0);
  }
};

// Soft circular-obstacle penalty for planar robots (positions in state rows
// 0/1). Penalties are soft by nature — safety-critical constraints need an
// output-stage shield on top (see the technical note).
struct CircularObstacleCost {
  using State = Eigen::Matrix<double, 3, 1>;
  using Control = Eigen::Matrix<double, 2, 1>;

  struct Obstacle {
    Eigen::Vector2d center;
    double radius;
  };

  std::vector<Obstacle> obstacles;
  double weight = 500.0;
  double margin = 0.1;

  double StageCost(const State &x, const Control & /*u*/, int /*t*/) const {
    double cost = 0.0;
    const Eigen::Vector2d p = x.head<2>();
    for (const auto &ob : obstacles) {
      const double clearance = (p - ob.center).norm() - ob.radius - margin;
      if (clearance < 0.0) {
        cost += weight * clearance * clearance;
      }
    }
    return cost;
  }
  double TerminalCost(const State & /*x*/) const { return 0.0; }
};

// Compile-time sum of critics.
template <typename... Critics>
class CompositeCost {
 public:
  explicit CompositeCost(Critics... critics)
      : critics_(std::move(critics)...) {}

  template <typename State, typename Control>
  double StageCost(const State &x, const Control &u, int t) const {
    return std::apply(
        [&](const auto &...critic) {
          return (critic.StageCost(x, u, t) + ...);
        },
        critics_);
  }
  template <typename State>
  double TerminalCost(const State &x) const {
    return std::apply(
        [&](const auto &...critic) { return (critic.TerminalCost(x) + ...); },
        critics_);
  }

 private:
  std::tuple<Critics...> critics_;
};

template <typename... Critics>
CompositeCost<Critics...> MakeCompositeCost(Critics... critics) {
  return CompositeCost<Critics...>(std::move(critics)...);
}

}  // namespace xmotion

#endif  // XMNAV_MPPI_CRITICS_HPP
