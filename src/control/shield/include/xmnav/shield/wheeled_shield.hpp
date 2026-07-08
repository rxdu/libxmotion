/*
 * @file wheeled_shield.hpp
 * @brief Output-stage safety shield for a diff-drive platform.
 *
 * Composes the three shield layers (docs/control/safety_shield.md) into
 * the object a robot application puts between the controller and the
 * actuator write:
 *
 *   auto u = shield.Filter(mppi.Command(), state, state_age, dt);
 *
 * Per tick: validate inputs at the boundary (finite command/state, fresh
 * state) -> barrier-filter the command (hard obstacle constraint) ->
 * degradation ladder -> envelope (box + rate limits) on the passthrough
 * path. Faults never throw on this path; they degrade through
 * Hold -> Stopping -> Stopped and are visible in LastReport() and in
 * telemetry (control.shield.*).
 *
 * TriggerEStop() takes effect on the next Filter() call; it complements,
 * never replaces, the hardware e-stop chain. Reset() re-arms from
 * kStopped only if the last tick's inputs were valid.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_SHIELD_WHEELED_SHIELD_HPP
#define XMNAV_SHIELD_WHEELED_SHIELD_HPP

#include <eigen3/Eigen/Dense>

#include "xmbase/telemetry/telemetry.hpp"
#include "xmnav/shield/command_envelope.hpp"
#include "xmnav/shield/diff_drive_barrier.hpp"
#include "xmnav/shield/fallback_ladder.hpp"
#include "xmnav/shield/report.hpp"

namespace xmotion {

class WheeledShield {
 public:
  static constexpr int kControlDim = 2;
  using Control = Eigen::Vector2d;  // [v m/s, omega rad/s]
  using State = Eigen::Vector3d;    // [x m, y m, theta rad] world frame

  struct Config {
    CommandEnvelope<kControlDim> envelope;
    DiffDriveBarrierFilter::Config barrier;
    bool enable_barrier = true;
    FallbackLadder::Config ladder;
    // state estimates older than this are faults (S4)
    double state_staleness_max = 0.2;  // s
  };

  explicit WheeledShield(const Config &config)
      : config_(config), ladder_(config.ladder), barrier_(config.barrier) {}

  // One control tick: returns the command to actuate. Never throws.
  Control Filter(const Control &u_raw, const State &state, double state_age,
                 double dt) {
    report_ = ShieldReport{};
    const bool inputs_valid = u_raw.allFinite() && state.allFinite() &&
                              state_age >= 0.0 &&
                              state_age <= config_.state_staleness_max &&
                              dt > 0.0;
    report_.input_valid = inputs_valid;

    Control candidate = Control::Zero();
    if (inputs_valid) {
      candidate = config_.enable_barrier
                      ? barrier_.Filter(state, u_raw, &report_)
                      : u_raw;
    }
    const bool fault = !inputs_valid || report_.barrier_infeasible;
    if (fault) {
      ladder_.OnFault();
      faults_.Add(1.0);
    } else {
      ladder_.OnRecovered();  // no-op unless the ladder is in kHold
    }
    ladder_.Tick(dt);

    Control out = Control::Zero();
    switch (ladder_.mode()) {
      case ShieldMode::kNormal: {
        bool clamped = false;
        out = config_.envelope.Apply(candidate, last_issued_, dt, &clamped);
        report_.envelope_active = clamped;
        held_ = out;  // the command Hold falls back to
        break;
      }
      case ShieldMode::kHold:
        out = held_;
        break;
      case ShieldMode::kStopping:
        out = held_ * ladder_.RampScale();
        break;
      case ShieldMode::kStopped:
        out = Control::Zero();
        break;
    }

    last_inputs_valid_ = inputs_valid && !report_.barrier_infeasible;
    report_.mode = ladder_.mode();
    report_.modified =
        !inputs_valid || (out - u_raw).cwiseAbs().maxCoeff() > 1e-12;
    last_issued_ = out;

    mode_gauge_.Set(static_cast<double>(report_.mode));
    if (report_.barrier_active) barrier_engaged_.Add(1.0);
    return out;
  }

  void TriggerEStop() { ladder_.TriggerEStop(); }

  // re-arm from kStopped; accepted only if the last tick's inputs were
  // valid (guarded in the ladder's transition table)
  bool Reset() { return ladder_.Reset(last_inputs_valid_); }

  ShieldMode mode() const { return ladder_.mode(); }
  const ShieldReport &LastReport() const { return report_; }

  // live access (e.g. obstacle updates from the mapping layer)
  DiffDriveBarrierFilter &barrier() { return barrier_; }

 private:
  Config config_;
  FallbackLadder ladder_;
  DiffDriveBarrierFilter barrier_;
  Control last_issued_ = Control::Zero();
  Control held_ = Control::Zero();
  bool last_inputs_valid_ = false;
  ShieldReport report_;

  telemetry::Gauge mode_gauge_ = telemetry::GetGauge("control.shield.mode");
  telemetry::Counter faults_ =
      telemetry::GetCounter("control.shield.faults");
  telemetry::Counter barrier_engaged_ =
      telemetry::GetCounter("control.shield.barrier_active");
};

}  // namespace xmotion

#endif  // XMNAV_SHIELD_WHEELED_SHIELD_HPP
