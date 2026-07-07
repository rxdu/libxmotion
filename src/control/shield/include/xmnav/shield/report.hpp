/*
 * @file report.hpp
 * @brief Per-tick report of what the safety shield did.
 *
 * A shield that silently edits commands is an observability hole: every
 * Filter() call fills one of these, and the facade mirrors the highlights
 * into telemetry (docs/control/safety_shield.md).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_SHIELD_REPORT_HPP
#define XMNAV_SHIELD_REPORT_HPP

#include <limits>

namespace xmotion {

enum class ShieldMode {
  kNormal = 0,   // passing (filtered) controller commands through
  kHold = 1,     // fault: issuing the last safe command
  kStopping = 2, // fault persisted: ramping the held command to zero
  kStopped = 3,  // latched at zero; explicit Reset() required
};

struct ShieldReport {
  ShieldMode mode = ShieldMode::kNormal;
  // the issued command differs from the raw controller command
  bool modified = false;
  // raw command + state were finite and fresh this tick
  bool input_valid = true;
  // box/rate envelope engaged
  bool envelope_active = false;
  // barrier constraint modified the command
  bool barrier_active = false;
  // barrier QP had no feasible command (treated as a fault)
  bool barrier_infeasible = false;
  // smallest obstacle clearance seen by the barrier this tick [m];
  // +inf when no obstacle is inside the influence radius
  double min_clearance = std::numeric_limits<double>::infinity();
};

}  // namespace xmotion

#endif  // XMNAV_SHIELD_REPORT_HPP
