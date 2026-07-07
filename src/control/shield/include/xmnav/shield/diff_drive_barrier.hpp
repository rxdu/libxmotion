/*
 * @file diff_drive_barrier.hpp
 * @brief Control-barrier-function command filter for a diff-drive robot
 *        among circular obstacles.
 *
 * The hard counterpart of the soft CircularObstacleCost: minimally modify
 * the commanded [v, omega] so that h_i(x) = ||p_l - o_i||^2 - D_i^2 obeys
 * dh_i/dt >= -alpha h_i for every obstacle, where p_l is the look-ahead
 * point l ahead of the axle (the standard relative-degree fix that makes
 * the constraint linear in BOTH v and omega for a unicycle). Guaranteeing
 * the look-ahead point keeps distance D + l guarantees the axle keeps D,
 * so D_i = r_i + robot_radius + margin + look_ahead.
 *
 * The filter is a 2-variable QP (min ||u - u_des||^2 s.t. the active
 * barrier rows + the actuator box), solved with the vendored QuadProg++
 * (Goldfarb-Idnani, MIT). Fast path: when u_des already satisfies every
 * row, no QP runs and the command passes through untouched (minimal
 * intervention, scenario S3). Known limitation: QuadProg++ allocates
 * internally per solve — µs-scale at this size; replace with a
 * preallocated solver if it ever shows in traces.
 *
 * A QP with no feasible command (e.g. already inside an obstacle) is
 * reported via ShieldReport::barrier_infeasible and must be treated as a
 * fault by the caller — it is never silently passed through.
 *
 * Units: state [x m, y m, theta rad] world frame; command [v m/s,
 * omega rad/s].
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_SHIELD_DIFF_DRIVE_BARRIER_HPP
#define XMNAV_SHIELD_DIFF_DRIVE_BARRIER_HPP

#include <cmath>
#include <limits>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "quadprog++/QuadProg++.hh"

#include "xmnav/shield/report.hpp"

namespace xmotion {

class DiffDriveBarrierFilter {
 public:
  struct Obstacle {
    Eigen::Vector2d center;
    double radius;
  };

  struct Config {
    double look_ahead = 0.15;  // l > 0 [m]
    double alpha = 2.0;        // class-K gain [1/s]
    double margin = 0.10;      // extra clearance beyond the radii [m]
    double robot_radius = 0.0;
    // obstacles farther than this clearance are skipped entirely
    double influence_distance = 2.0;  // [m]
    // actuator box (mirror the controller's params.u_min/u_max)
    Eigen::Vector2d u_min{-std::numeric_limits<double>::infinity(),
                          -std::numeric_limits<double>::infinity()};
    Eigen::Vector2d u_max{std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::infinity()};
    std::vector<Obstacle> obstacles;
  };

  explicit DiffDriveBarrierFilter(Config config)
      : config_(std::move(config)) {}

  const Config &config() const { return config_; }
  Config &config() { return config_; }

  // Minimally modified command; fills the barrier fields of *report.
  Eigen::Vector2d Filter(const Eigen::Vector3d &state,
                         const Eigen::Vector2d &u_des,
                         ShieldReport *report) const {
    const double c = std::cos(state(2));
    const double s = std::sin(state(2));
    const double l = config_.look_ahead;
    const Eigen::Vector2d p(state(0), state(1));
    const Eigen::Vector2d p_l = p + l * Eigen::Vector2d(c, s);
    // dp_l/dt = M(theta) [v, omega]
    Eigen::Matrix2d M;
    M << c, -l * s, s, l * c;

    // collect active rows a^T u >= -alpha h
    std::vector<Eigen::Vector2d> rows;
    std::vector<double> offsets;  // ci0 = alpha h
    rows.reserve(config_.obstacles.size());
    offsets.reserve(config_.obstacles.size());
    double min_clearance = std::numeric_limits<double>::infinity();
    bool all_satisfied = true;
    for (const auto &ob : config_.obstacles) {
      const double clearance =
          (p - ob.center).norm() - ob.radius - config_.robot_radius;
      min_clearance = std::min(min_clearance, clearance);
      if (clearance > config_.influence_distance) continue;
      const double safe_dist =
          ob.radius + config_.robot_radius + config_.margin + l;
      const Eigen::Vector2d d = p_l - ob.center;
      const double h = d.squaredNorm() - safe_dist * safe_dist;
      const Eigen::Vector2d a = 2.0 * M.transpose() * d;
      rows.push_back(a);
      offsets.push_back(config_.alpha * h);
      if (a.dot(u_des) + config_.alpha * h < 0.0) all_satisfied = false;
    }
    if (report != nullptr) {
      report->min_clearance = min_clearance;
      report->barrier_active = false;
      report->barrier_infeasible = false;
    }
    if (rows.empty() || all_satisfied) {
      return u_des;  // minimal intervention: nothing to do
    }

    // QP: min 1/2 u^T I u - u_des^T u  s.t.  CI^T u + ci0 >= 0
    const int n = 2;
    const int m = static_cast<int>(rows.size()) + 4;  // rows + box
    quadprogpp::Matrix<double> G(n, n), CE(n, 0), CI(n, m);
    quadprogpp::Vector<double> g0(n), ce0(0), ci0(m), x(n);
    G[0][0] = 1.0; G[0][1] = 0.0; G[1][0] = 0.0; G[1][1] = 1.0;
    g0[0] = -u_des(0);
    g0[1] = -u_des(1);
    int col = 0;
    for (std::size_t i = 0; i < rows.size(); ++i, ++col) {
      CI[0][col] = rows[i](0);
      CI[1][col] = rows[i](1);
      ci0[col] = offsets[i];
    }
    for (int j = 0; j < 2; ++j) {
      // u_j >= u_min_j and -u_j >= -u_max_j (skip infinities via huge ci0)
      CI[0][col] = (j == 0) ? 1.0 : 0.0;
      CI[1][col] = (j == 1) ? 1.0 : 0.0;
      ci0[col] = std::isfinite(config_.u_min(j)) ? -config_.u_min(j) : 1e30;
      ++col;
      CI[0][col] = (j == 0) ? -1.0 : 0.0;
      CI[1][col] = (j == 1) ? -1.0 : 0.0;
      ci0[col] = std::isfinite(config_.u_max(j)) ? config_.u_max(j) : 1e30;
      ++col;
    }
    const double cost =
        quadprogpp::solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
    if (!std::isfinite(cost) || !std::isfinite(x[0]) ||
        !std::isfinite(x[1])) {
      if (report != nullptr) report->barrier_infeasible = true;
      return u_des;  // caller must treat this tick as a fault
    }
    if (report != nullptr) report->barrier_active = true;
    return Eigen::Vector2d(x[0], x[1]);
  }

 private:
  Config config_;
};

}  // namespace xmotion

#endif  // XMNAV_SHIELD_DIFF_DRIVE_BARRIER_HPP
