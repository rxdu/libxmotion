/*
 * @file wheeled_program.hpp
 * @brief Device-side rollout program for the wheeled (diff-drive) platform.
 *
 * A "program" is the POD the CUDA rollout kernel reads (from device
 * memory): the model + cost of one MPPI configuration flattened into
 * trivially copyable storage, with Step/StageCost/TerminalCost methods
 * that call the same scalar cores the CPU classes wrap (model_core /
 * critic_core). One program type per platform family (srb_program.hpp is
 * the quadruped's).
 *
 * The device computes in float — FP64 runs at 1/32 rate on GTX-class and
 * Jetson Orin hardware, and sampling MPC does not need double rollouts.
 * The host backend narrows the double-typed configuration when building
 * the program.
 *
 * Compiled by nvcc: keep the include surface to the scalar cores only.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MPPI_CUDA_WHEELED_PROGRAM_HPP
#define XMNAV_MPPI_CUDA_WHEELED_PROGRAM_HPP

#include <type_traits>

#include "xmnav/mppi/critic_core.hpp"
#include "xmnav/models/device.hpp"
#include "xmnav/models/model_core.hpp"

namespace xmotion {

struct WheeledProgram {
  static constexpr int kStateDim = 3;
  static constexpr int kControlDim = 2;
  static constexpr int kMaxObstacles = 16;

  // Se2GoalCost
  float goal[3];
  float position_weight;
  float heading_weight;
  float terminal_scale;
  // CircularObstacleCost, flattened (x, y, r) triples
  float obstacles[kMaxObstacles * 3];
  int num_obstacles;
  float obstacle_weight;
  float obstacle_margin;

  XMNAV_HD void Step(const float x[3], const float u[2], int /*t*/, float dt,
                     float next[3]) const {
    model_core::DiffDriveStep(x, u, dt, next);
  }

  XMNAV_HD float StageCost(const float x[3], const float /*u*/[2],
                           int /*t*/) const {
    float c = critic_core::Se2GoalStage(x, goal, position_weight,
                                        heading_weight);
    for (int i = 0; i < num_obstacles; ++i) {
      c += critic_core::CircularObstaclePenalty(
          x[0] - obstacles[3 * i], x[1] - obstacles[3 * i + 1],
          obstacles[3 * i + 2], obstacle_margin, obstacle_weight);
    }
    return c;
  }

  XMNAV_HD float TerminalCost(const float x[3]) const {
    return terminal_scale * critic_core::Se2GoalStage(x, goal,
                                                      position_weight,
                                                      heading_weight);
  }
};

static_assert(std::is_trivially_copyable<WheeledProgram>::value,
              "device program must be trivially copyable");

}  // namespace xmotion

#endif  // XMNAV_MPPI_CUDA_WHEELED_PROGRAM_HPP
