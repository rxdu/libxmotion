/*
 * @file srb_program.hpp
 * @brief Device-side rollout program for the SRB quadruped platform.
 *
 * Flattens one SRB MPPI configuration — trunk parameters, per-horizon-step
 * foot positions and contact schedule, and the tracking / friction-cone /
 * control-regularization critics — into a trivially copyable POD whose
 * methods call the same scalar cores the CPU classes wrap. The kernel
 * reads it from device memory (at ~3.5 KB it is too large for a kernel
 * argument).
 *
 * The single stance table serves both the dynamics (swing-force masking)
 * and the friction-cone critic, mirroring how tests wire the same schedule
 * into both; the host backend validates that the two host-side schedules
 * agree before building the program. Control regularization is restricted
 * to diagonal R on the device (validated fail-loud) — the in-tree usage is
 * diagonal.
 *
 * Compiled by nvcc: keep the include surface to the scalar cores only.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MPPI_CUDA_SRB_PROGRAM_HPP
#define XMNAV_MPPI_CUDA_SRB_PROGRAM_HPP

#include <type_traits>

#include "xmnav/mppi/critic_core.hpp"
#include "xmnav/mppi/device.hpp"
#include "xmnav/mppi/model_core.hpp"

namespace xmotion {

struct SrbProgram {
  static constexpr int kStateDim = 13;
  static constexpr int kControlDim = 12;
  static constexpr int kNumFeet = 4;
  static constexpr int kMaxHorizon = 64;

  // trunk (SrbQuadrupedModel::Params)
  float mass;
  float gravity;
  float inertia_diag[3];
  // per-horizon-step context, pre-expanded on the host (FeetAt/InStance
  // clamping already applied)
  float feet[kMaxHorizon * 3 * kNumFeet];
  unsigned char stance[kMaxHorizon * kNumFeet];
  // SrbTrackingCost
  float height_ref;
  float velocity_ref[3];
  float height_weight;
  float tilt_weight;
  float velocity_weight;
  float angular_rate_weight;
  float terminal_scale;
  // FrictionConeCost
  float mu;
  float cone_weight;
  // QuadraticControlCost, diagonal R
  float r_diag[kControlDim];

  XMNAV_HD void Step(const float x[13], const float u[12], int t, float dt,
                     float next[13]) const {
    model_core::SrbQuadrupedStep(x, u, feet + 3 * kNumFeet * t,
                                 stance + kNumFeet * t, inertia_diag, mass,
                                 gravity, dt, next);
  }

  XMNAV_HD float StageCost(const float x[13], const float u[12],
                           int t) const {
    // critic order matches the CPU composite: tracking, cone, regulation
    float c = critic_core::SrbTrackingStage(
        x, height_ref, velocity_ref, height_weight, tilt_weight,
        velocity_weight, angular_rate_weight);
    for (int i = 0; i < kNumFeet; ++i) {
      if (stance[kNumFeet * t + i]) {
        c += critic_core::FrictionConePenalty(u + 3 * i, mu, cone_weight);
      }
    }
    float reg = 0.0f;
    for (int j = 0; j < kControlDim; ++j) reg += r_diag[j] * u[j] * u[j];
    return c + reg;
  }

  XMNAV_HD float TerminalCost(const float x[13]) const {
    return terminal_scale * critic_core::SrbTrackingStage(
                                x, height_ref, velocity_ref, height_weight,
                                tilt_weight, velocity_weight,
                                angular_rate_weight);
  }
};

static_assert(std::is_trivially_copyable<SrbProgram>::value,
              "device program must be trivially copyable");

}  // namespace xmotion

#endif  // XMNAV_MPPI_CUDA_SRB_PROGRAM_HPP
