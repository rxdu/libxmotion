/*
 * demo_srb_quadruped.cpp
 *
 * Ground-plane view of the SRB quadruped trotting under MPPI GRF control:
 * candidate trunk trajectories with weight intensity, the executed trail,
 * and the current stance feet.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <cstdio>

#include "xmnav/mppi/critics.hpp"
#include "xmnav/mppi/critics_srb.hpp"
#include "xmnav/models/srb_quadruped.hpp"
#include "xmnav/mppi/mppi.hpp"
#include "xmnav/sim/sim_loop.hpp"
#include "xmnav/viz/mppi_draw.hpp"
#include "xmnav/viz/sim_viewer.hpp"

using namespace xmotion;
using Srb = SrbQuadrupedModel;

namespace {
const std::array<Eigen::Vector3d, 4> kOffsets = {
    Eigen::Vector3d(0.19, 0.11, 0.0), Eigen::Vector3d(0.19, -0.11, 0.0),
    Eigen::Vector3d(-0.19, 0.11, 0.0), Eigen::Vector3d(-0.19, -0.11, 0.0)};
constexpr int kHorizon = 20;
constexpr double kDt = 0.02;
constexpr int kPhase = 10;

std::array<bool, 4> PhaseOf(int step) {
  const bool a = ((step / kPhase) % 2) == 0;
  return {a, !a, !a, a};
}
Srb::ContactSchedule Schedule(int step) {
  Srb::ContactSchedule s(kHorizon);
  for (int t = 0; t < kHorizon; ++t) s[t] = PhaseOf(step + t);
  return s;
}
}  // namespace

int main() {
  SrbTrackingCost tracking;
  tracking.height_ref = 0.28;
  tracking.velocity_ref = Eigen::Vector3d(0.3, 0.0, 0.0);
  tracking.velocity_weight = 250.0;
  FrictionConeCost cone;
  QuadraticControlCost<13, 12> reg;
  reg.R = Eigen::Matrix<double, 12, 12>::Identity() * 1e-4;
  auto cost = MakeCompositeCost(tracking, cone, reg);

  using Controller = Mppi<Srb, decltype(cost), SplineKnotSampler<12>>;
  Controller::Params p;
  p.num_samples = 2048;
  p.horizon_steps = kHorizon;
  p.dt = kDt;
  p.lambda = 0.1;
  p.control_cost_decoupling = 1.0;
  p.normalize_cost_spread = true;
  for (int i = 0; i < 4; ++i) {
    p.sigma.segment<3>(3 * i) << 8.0, 8.0, 15.0;
    p.u_min.segment<3>(3 * i) << -60.0, -60.0, 0.0;
    p.u_max.segment<3>(3 * i) << 60.0, 60.0, 160.0;
  }
  Controller mppi(Srb{}, cost, p, SplineKnotSampler<12>(11, 5));
  Srb::Control seed = Srb::Control::Zero();
  for (int i = 0; i < 4; ++i) seed(3 * i + 2) = 15.0 * 9.81 / 4.0;
  mppi.SeedSequence(seed);
  mppi.EnableIntrospection(16, 16);

  SimViewer2D::Config vc;
  vc.x_min = -0.5;
  vc.x_max = 3.5;
  vc.y_min = -1.0;
  vc.y_max = 1.0;
  vc.window_name = "srb quadruped (ground plane)";
  SimViewer2D viewer(vc);
  Srb::FootPositions feet;
  std::array<bool, 4> stance{};
  viewer.AddOverlay([&](quickviz::CvCanvas &canvas) {
    DrawMppiSnapshot2D(canvas, mppi.LastSnapshot());
    for (int i = 0; i < 4; ++i) {
      canvas.DrawPoint({feet[i](0), feet[i](1)}, stance[i] ? 4 : 2,
                       stance[i] ? quickviz::CvColors::black_color
                                 : quickviz::CvColors::gray_color);
    }
  });

  Srb plant;
  Srb::State x = Srb::MakeState({0, 0, 0.28}, {0, 0, 0},
                                Eigen::Quaterniond::Identity(), {0, 0, 0});
  for (std::size_t i = 0; i < 4; ++i) {
    feet[i] = Eigen::Vector3d(0, 0, 0) + kOffsets[i];
  }

  for (int i = 0; i < 500; ++i) {
    const auto sch = Schedule(i);
    stance = PhaseOf(i);
    // simple gait context: touchdown at hip projection + half-stance lead
    const auto now = PhaseOf(i);
    const auto next = PhaseOf(i + 1);
    for (std::size_t f = 0; f < 4; ++f) {
      if (!now[f] && next[f]) {
        feet[f] = Eigen::Vector3d(Srb::Position(x)(0), Srb::Position(x)(1),
                                  0.0) +
                  kOffsets[f] +
                  tracking.velocity_ref * (0.5 * kPhase * kDt);
      }
    }
    std::vector<Srb::FootPositions> plan(kHorizon, feet);
    mppi.model().SetContext(plan, sch);
    plant.SetContext(feet, sch);
    mppi.Plan(x);
    x = plant.Step(x, mppi.Command(), 0, kDt);
    viewer.Frame(Srb::Position(x)(0), Srb::Position(x)(1));
    if (i % 50 == 0) {
      std::printf("t=%4.1fs  v=%.2f m/s  h=%.3f m  ESS=%.0f\n", i * kDt,
                  Srb::Velocity(x)(0), Srb::Position(x)(2),
                  mppi.LastEffectiveSampleSize());
    }
  }
  return 0;
}
