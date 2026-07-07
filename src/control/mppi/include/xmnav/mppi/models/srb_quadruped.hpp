/*
 * @file srb_quadruped.hpp
 * @brief Single-rigid-body quadruped model for MPPI rollouts.
 *
 * The reduced-order model of choice for sampling MPC on legged platforms
 * (Turrisi et al. 2024, following the convex-MPC lineage of Di Carlo/Kim):
 * the trunk is one rigid body driven by per-foot ground reaction forces;
 * legs are massless force transmitters. The controller samples GRF
 * trajectories (typically through SplineKnotSampler); a downstream leg
 * controller maps stance forces to joint torques (tau = -J^T f) and swings
 * the flight legs — that layer is application/hardware territory (ADR 0005).
 *
 * State (13): [p(3), v(3), q(4, wxyz body->world), omega(3, world frame)]
 * Control (12): [f_LF(3), f_RF(3), f_LH(3), f_RH(3)] world-frame GRFs
 *
 * The contact schedule and world-frame foot positions are per-cycle context
 * owned by the model instance: the application (or a test's gait generator)
 * updates them through the controller's model() accessor before each Plan().
 * Swing feet transmit no force: their sampled controls are masked to zero
 * inside Step(), which also keeps the weighted update from accumulating
 * force on unloaded feet.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MPPI_MODELS_SRB_QUADRUPED_HPP
#define XMNAV_MPPI_MODELS_SRB_QUADRUPED_HPP

#include <array>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace xmotion {

class SrbQuadrupedModel {
 public:
  static constexpr int kStateDim = 13;
  static constexpr int kControlDim = 12;
  static constexpr int kNumFeet = 4;

  using State = Eigen::Matrix<double, kStateDim, 1>;
  using Control = Eigen::Matrix<double, kControlDim, 1>;
  using FootPositions = std::array<Eigen::Vector3d, kNumFeet>;
  // contact_schedule[t][i]: is foot i in stance at horizon step t
  using ContactSchedule = std::vector<std::array<bool, kNumFeet>>;

  struct Params {
    double mass = 15.0;                                   // kg
    Eigen::Vector3d inertia_diag{0.07, 0.26, 0.28};       // trunk, body frame
    double gravity = 9.81;
  };

  SrbQuadrupedModel() : params_(Params{}) {}
  explicit SrbQuadrupedModel(const Params &params) : params_(params) {}

  // uniform context: same foot positions across the horizon (standing)
  void SetContext(const FootPositions &feet, ContactSchedule schedule) {
    feet_plan_.assign(1, feet);
    schedule_ = std::move(schedule);
  }
  // per-step context: foot positions per horizon step, including planned
  // touchdown locations for future stance phases (locomotion). Without
  // forward touchdown placement, rollouts see stance feet falling behind
  // the advancing trunk and price speed as attitude risk.
  void SetContext(std::vector<FootPositions> feet_plan,
                  ContactSchedule schedule) {
    feet_plan_ = std::move(feet_plan);
    schedule_ = std::move(schedule);
  }

  const FootPositions &FeetAt(int t) const {
    const std::size_t idx = std::min(
        static_cast<std::size_t>(t < 0 ? 0 : t), feet_plan_.size() - 1);
    return feet_plan_[idx];
  }

  const Params &params() const { return params_; }

  // state accessors (column layout documented above)
  static Eigen::Vector3d Position(const State &x) { return x.segment<3>(0); }
  static Eigen::Vector3d Velocity(const State &x) { return x.segment<3>(3); }
  static Eigen::Quaterniond Orientation(const State &x) {
    return Eigen::Quaterniond(x(6), x(7), x(8), x(9));
  }
  static Eigen::Vector3d AngularVelocity(const State &x) {
    return x.segment<3>(10);
  }
  static State MakeState(const Eigen::Vector3d &p, const Eigen::Vector3d &v,
                         const Eigen::Quaterniond &q,
                         const Eigen::Vector3d &omega) {
    State x;
    x.segment<3>(0) = p;
    x.segment<3>(3) = v;
    x(6) = q.w();
    x(7) = q.x();
    x(8) = q.y();
    x(9) = q.z();
    x.segment<3>(10) = omega;
    return x;
  }

  bool InStance(int t, int foot) const {
    if (schedule_.empty()) return true;
    const std::size_t idx =
        std::min(static_cast<std::size_t>(t < 0 ? 0 : t), schedule_.size() - 1);
    return schedule_[idx][static_cast<std::size_t>(foot)];
  }

  State Step(const State &x, const Control &u, int t, double dt) const {
    const Eigen::Vector3d p = Position(x);
    const Eigen::Vector3d v = Velocity(x);
    Eigen::Quaterniond q = Orientation(x);
    q.normalize();
    const Eigen::Vector3d omega = AngularVelocity(x);

    // total force and torque about the CoM from stance feet only
    Eigen::Vector3d force_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d torque_sum = Eigen::Vector3d::Zero();
    const FootPositions &feet = FeetAt(t);
    for (int i = 0; i < kNumFeet; ++i) {
      if (!InStance(t, i)) continue;  // swing feet transmit nothing
      const Eigen::Vector3d f = u.segment<3>(3 * i);
      force_sum += f;
      torque_sum += (feet[static_cast<std::size_t>(i)] - p).cross(f);
    }

    const Eigen::Matrix3d R = q.toRotationMatrix();
    const Eigen::Matrix3d I_w =
        R * params_.inertia_diag.asDiagonal() * R.transpose();

    const Eigen::Vector3d accel =
        force_sum / params_.mass + Eigen::Vector3d(0, 0, -params_.gravity);
    const Eigen::Vector3d omega_dot =
        I_w.ldlt().solve(torque_sum - omega.cross(I_w * omega));

    // integrate (forward Euler; quaternion via the omega increment)
    Eigen::Quaterniond dq(1.0, 0.5 * omega(0) * dt, 0.5 * omega(1) * dt,
                          0.5 * omega(2) * dt);
    Eigen::Quaterniond q_next = (dq * q).normalized();  // world-frame omega

    return MakeState(p + v * dt, v + accel * dt, q_next,
                     omega + omega_dot * dt);
  }

 private:
  Params params_;
  std::vector<FootPositions> feet_plan_{1};
  ContactSchedule schedule_{};
};

}  // namespace xmotion

#endif  // XMNAV_MPPI_MODELS_SRB_QUADRUPED_HPP
