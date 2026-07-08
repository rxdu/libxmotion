/*
 * @file fallback_ladder.hpp
 * @brief Degradation ladder of the safety shield: Normal -> Hold ->
 *        Stopping -> Stopped, with an e-stop wildcard.
 *
 * Built on the vendored ctfsm engine (its first in-tree consumer): the
 * transition table below is the complete legal graph, verified at compile
 * time; dispatch is allocation-free and an event with no row in the
 * current state is refused, which is exactly the policy encoding — e.g.
 * InputRecovered is dispatched every healthy tick and only does something
 * in kHold (no automatic recovery from kStopping/kStopped; a guarded
 * operator Reset() re-arms). Scenarios S2/S4/S6 of
 * docs/control/safety_shield.md.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_SHIELD_FALLBACK_LADDER_HPP
#define XMNAV_SHIELD_FALLBACK_LADDER_HPP

#include <algorithm>

#include "ctfsm/fsm.hpp"

#include "xmnav/shield/report.hpp"

namespace xmotion {

namespace shield_detail {

struct LadderContext {
  ShieldMode mode = ShieldMode::kNormal;
  double hold_timeout = 0.3;    // s in kHold before the controlled stop
  double stop_ramp_time = 0.5;  // s to ramp the held command to zero
  double dt = 0.0;              // set by Tick() before the FSM update
  double hold_elapsed = 0.0;
  double ramp_elapsed = 0.0;
  bool reset_allowed = false;   // set by the facade before Reset dispatch
};

struct Normal {
  void OnEnter(LadderContext &c) { c.mode = ShieldMode::kNormal; }
};
struct Hold {
  void OnEnter(LadderContext &c) {
    c.mode = ShieldMode::kHold;
    c.hold_elapsed = 0.0;
  }
  void Update(LadderContext &c) { c.hold_elapsed += c.dt; }
};
struct Stopping {
  void OnEnter(LadderContext &c) {
    c.mode = ShieldMode::kStopping;
    c.ramp_elapsed = 0.0;
  }
  void Update(LadderContext &c) { c.ramp_elapsed += c.dt; }
};
struct Stopped {
  void OnEnter(LadderContext &c) { c.mode = ShieldMode::kStopped; }
};

// events
struct FaultDetected {};
struct InputRecovered {};
struct HoldExpired {};
struct RampComplete {};
struct EStopEvent {};
struct ResetEvent {};

struct ResetAllowed {
  bool operator()(const LadderContext &c) const noexcept {
    return c.reset_allowed;
  }
};

// clang-format off
using LadderMachine = ctfsm::StateMachine<LadderContext,
    ctfsm::StateList<Normal, Hold, Stopping, Stopped>,
    ctfsm::Table<
      //          From             Event           To        Guard
      ctfsm::Row< Normal,          FaultDetected,  Hold                    >,
      ctfsm::Row< Hold,            InputRecovered, Normal                  >,
      ctfsm::Row< Hold,            HoldExpired,    Stopping                >,
      ctfsm::Row< Stopping,        RampComplete,   Stopped                 >,
      ctfsm::Row< ctfsm::AnyState, EStopEvent,     Stopped                 >,
      ctfsm::Row< Stopped,         ResetEvent,     Normal,   ResetAllowed  >>>;
// clang-format on

}  // namespace shield_detail

class FallbackLadder {
 public:
  struct Config {
    double hold_timeout = 0.3;    // s
    double stop_ramp_time = 0.5;  // s
  };

  explicit FallbackLadder(const Config &config) {
    ctx_.hold_timeout = config.hold_timeout;
    ctx_.stop_ramp_time = config.stop_ramp_time;
    fsm_.Start(ctx_);
  }

  void OnFault() { fsm_.Dispatch(shield_detail::FaultDetected{}, ctx_); }
  void OnRecovered() {
    fsm_.Dispatch(shield_detail::InputRecovered{}, ctx_);
  }
  void TriggerEStop() { fsm_.Dispatch(shield_detail::EStopEvent{}, ctx_); }

  // guarded re-arm; the caller states whether inputs are currently valid
  bool Reset(bool inputs_valid) {
    ctx_.reset_allowed = inputs_valid;
    const bool accepted = fsm_.Dispatch(shield_detail::ResetEvent{}, ctx_);
    ctx_.reset_allowed = false;
    return accepted;
  }

  // advance timers and fire the timeout transitions
  void Tick(double dt) {
    ctx_.dt = dt;
    fsm_.Update(ctx_);
    if (ctx_.mode == ShieldMode::kHold &&
        ctx_.hold_elapsed >= ctx_.hold_timeout) {
      fsm_.Dispatch(shield_detail::HoldExpired{}, ctx_);
    }
    if (ctx_.mode == ShieldMode::kStopping &&
        ctx_.ramp_elapsed >= ctx_.stop_ramp_time) {
      fsm_.Dispatch(shield_detail::RampComplete{}, ctx_);
    }
  }

  ShieldMode mode() const { return ctx_.mode; }

  // scale in [0, 1] to apply to the held command while stopping
  double RampScale() const {
    if (ctx_.mode != ShieldMode::kStopping) {
      return ctx_.mode == ShieldMode::kHold ? 1.0 : 0.0;
    }
    return std::max(0.0, 1.0 - ctx_.ramp_elapsed / ctx_.stop_ramp_time);
  }

 private:
  shield_detail::LadderContext ctx_;
  shield_detail::LadderMachine fsm_;
};

}  // namespace xmotion

#endif  // XMNAV_SHIELD_FALLBACK_LADDER_HPP
