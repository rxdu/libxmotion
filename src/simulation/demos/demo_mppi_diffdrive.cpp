/*
 * demo_mppi_diffdrive.cpp
 *
 * Live inspection of the MPPI controller on a differential-drive robot:
 * candidate rollouts drawn with weight-encoded intensity, the chosen
 * trajectory in blue, obstacles and goal in the world frame.
 *
 * Run from the repo root; press any key per frame if frame_period_ms = 0,
 * Esc/close-window semantics follow OpenCV.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <cstdio>

#include "xmnav/mppi/critics.hpp"
#include "xmnav/mppi/models/diff_drive.hpp"
#include "xmnav/mppi/mppi.hpp"
#include "xmnav/sim/sim_loop.hpp"
#include "xmnav/viz/mppi_draw.hpp"
#include "xmnav/viz/sim_viewer.hpp"

using namespace xmotion;

int main() {
  Se2GoalCost goal_cost;
  goal_cost.goal << 3.5, 1.0, 0.0;
  CircularObstacleCost obstacle_cost;
  obstacle_cost.obstacles.push_back({Eigen::Vector2d(1.5, 0.2), 0.4});
  obstacle_cost.obstacles.push_back({Eigen::Vector2d(2.6, 1.1), 0.3});

  auto cost = MakeCompositeCost(goal_cost, obstacle_cost);
  using Controller = Mppi<DiffDriveModel, decltype(cost)>;
  Controller::Params p;
  p.num_samples = 1024;
  p.horizon_steps = 50;
  p.dt = 0.05;
  p.lambda = 0.3;
  p.sigma << 0.3, 0.8;
  p.u_min << -0.8, -2.0;
  p.u_max << 0.8, 2.0;
  p.normalize_cost_spread = true;
  Controller mppi(DiffDriveModel{}, cost, p);
  mppi.EnableIntrospection(24, 24);

  SimViewer2D::Config vc;
  vc.x_min = -0.5;
  vc.x_max = 4.5;
  vc.y_min = -1.0;
  vc.y_max = 2.0;
  SimViewer2D viewer(vc);
  for (const auto &ob : obstacle_cost.obstacles) {
    viewer.AddObstacle(ob.center, ob.radius);
  }
  viewer.SetGoal(goal_cost.goal.head<2>());
  viewer.AddOverlay([&](quickviz::CvCanvas &canvas) {
    DrawMppiSnapshot2D(canvas, mppi.LastSnapshot());
  });

  SimLoop<DiffDriveModel>::Config cfg;
  cfg.dt = 0.05;
  cfg.steps = 400;
  cfg.process_noise << 0.002, 0.002, 0.004;
  SimLoop<DiffDriveModel> sim(DiffDriveModel{}, cfg);

  sim.Run(
      DiffDriveModel::State::Zero(),
      [&](const DiffDriveModel::State &z, int) {
        mppi.Plan(z);
        return mppi.Command();
      },
      nullptr,
      [&](const DiffDriveModel::State &x, const DiffDriveModel::Control &,
          int t) {
        viewer.Frame(x(0), x(1));
        if (t % 50 == 0) {
          std::printf("t=%5.1fs  ESS=%5.0f  best=%.1f\n", t * cfg.dt,
                      mppi.LastEffectiveSampleSize(), mppi.LastBestCost());
        }
      });
  return 0;
}
