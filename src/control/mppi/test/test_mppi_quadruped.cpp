/*
 * test_mppi_quadruped.cpp
 *
 * The multi-platform proof point: the same MPPI core that drives the
 * differential-drive tests controls a single-rigid-body quadruped through
 * ground-reaction-force trajectories sampled as spline knots — the
 * architecture of Turrisi et al. 2024 (see docs/typst/mppi.typ). Only the
 * model, critics, and sampler configuration change; the core does not.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <cmath>

#include "xmnav/mppi/critics.hpp"
#include "xmnav/mppi/critics_srb.hpp"
#include "xmnav/mppi/models/srb_quadruped.hpp"
#include "xmnav/mppi/mppi.hpp"

using namespace xmotion;

namespace {

// Sanitizer/Debug builds exercise the code paths for memory errors but run
// the physics at reduced scale: full convergence workloads (12M rollout
// steps) take 100-200x longer under ASan and add nothing to what the
// Release suite already asserts numerically.
#if !defined(NDEBUG) || defined(__SANITIZE_ADDRESS__) ||     defined(__SANITIZE_THREAD__)
constexpr bool kReducedScale = true;
#else
constexpr bool kReducedScale = false;
#endif
constexpr int kSamples = kReducedScale ? 128 : 2048;
inline int Cycles(int full) { return kReducedScale ? 20 : full; }

constexpr double kDt = 0.02;
constexpr int kHorizon = 20;  // 0.4 s
constexpr double kHeight = 0.28;

using Srb = SrbQuadrupedModel;

// nominal foot offsets from the body center (LF, RF, LH, RH), x fwd / y left
const std::array<Eigen::Vector3d, 4> kFootOffsets = {
    Eigen::Vector3d(0.19, 0.11, 0.0), Eigen::Vector3d(0.19, -0.11, 0.0),
    Eigen::Vector3d(-0.19, 0.11, 0.0), Eigen::Vector3d(-0.19, -0.11, 0.0)};

Srb::FootPositions FeetUnderBody(const Eigen::Vector3d& p) {
  Srb::FootPositions feet;
  for (std::size_t i = 0; i < 4; ++i) {
    feet[i] = Eigen::Vector3d(p(0), p(1), 0.0) + kFootOffsets[i];
  }
  return feet;
}

Srb::ContactSchedule AllStance() {
  return Srb::ContactSchedule(kHorizon, {true, true, true, true});
}

// diagonal-pair trot: phase length in steps; cycle_step selects the phase
Srb::ContactSchedule TrotSchedule(int cycle_step, int phase_steps) {
  Srb::ContactSchedule s(kHorizon);
  for (int t = 0; t < kHorizon; ++t) {
    const bool diag_a = (((cycle_step + t) / phase_steps) % 2) == 0;
    // LF+RH vs RF+LH
    s[static_cast<std::size_t>(t)] = {diag_a, !diag_a, !diag_a, diag_a};
  }
  return s;
}

// Minimal gait generator for the trot test: keeps world-fixed anchors for
// stance feet and plans forward touchdown locations for future stance
// phases (Raibert-style hip-projection + half-stance lead). This is the
// application-layer context a real leg-control stack would provide.
class TrotGait {
 public:
  TrotGait(const Eigen::Vector3d& body, int phase_steps)
      : phase_steps_(phase_steps) {
    for (std::size_t i = 0; i < 4; ++i) {
      anchors_[i] = Eigen::Vector3d(body(0), body(1), 0.0) + kFootOffsets[i];
    }
  }

  // advance the executed gait by one control step; re-anchor feet that
  // just touched down at their planned location
  void Advance(int cycle_step, const Eigen::Vector3d& body,
               const Eigen::Vector3d& v_ref) {
    const auto now = PhaseOf(cycle_step);
    const auto next = PhaseOf(cycle_step + 1);
    for (std::size_t i = 0; i < 4; ++i) {
      if (!now[i] && next[i]) {  // touchdown at the next step
        anchors_[i] = Touchdown(body, v_ref, i);
      }
    }
  }

  // per-horizon-step foot plan for the controller, from current anchors,
  // with future touchdowns placed along the commanded velocity
  std::vector<Srb::FootPositions> Plan(int cycle_step,
                                       const Eigen::Vector3d& body,
                                       const Eigen::Vector3d& v_ref) const {
    std::vector<Srb::FootPositions> plan(kHorizon);
    Srb::FootPositions feet = anchors_;
    auto prev = PhaseOf(cycle_step);
    for (int t = 0; t < kHorizon; ++t) {
      const auto cur = PhaseOf(cycle_step + t);
      const Eigen::Vector3d body_pred = body + v_ref * (t * kDt);
      for (std::size_t i = 0; i < 4; ++i) {
        if (cur[i] && !prev[i]) feet[i] = Touchdown(body_pred, v_ref, i);
      }
      plan[static_cast<std::size_t>(t)] = feet;
      prev = cur;
    }
    return plan;
  }

 private:
  std::array<bool, 4> PhaseOf(int cycle_step) const {
    const bool a = ((cycle_step / phase_steps_) % 2) == 0;
    return {a, !a, !a, a};
  }
  Eigen::Vector3d Touchdown(const Eigen::Vector3d& body,
                            const Eigen::Vector3d& v_ref,
                            std::size_t foot) const {
    return Eigen::Vector3d(body(0), body(1), 0.0) + kFootOffsets[foot] +
           v_ref * (0.5 * phase_steps_ * kDt);  // half-stance lead
  }

  int phase_steps_;
  Srb::FootPositions anchors_;
};

struct Setup {
  using Cost = CompositeCost<SrbTrackingCost, FrictionConeCost,
                             QuadraticControlCost<13, 12>>;
  using Controller = Mppi<Srb, Cost, SplineKnotSampler<12>>;

  static Controller Make(const SrbTrackingCost& tracking,
                         const Srb::ContactSchedule& schedule,
                         std::uint64_t seed) {
    FrictionConeCost cone;
    cone.schedule = schedule;
    QuadraticControlCost<13, 12> reg;
    reg.R = Eigen::Matrix<double, 12, 12>::Identity() * 1e-4;
    Cost cost = MakeCompositeCost(tracking, cone, reg);

    Controller::Params p;
    // 12-dim GRF control needs coverage: 512 samples leaves 8-13 deg
    // attitude transients under disturbance; 2048 brings them under 5 deg
    // (see the sweep record in the PR). CPU cost: ~4 ms/plan.
    p.num_samples = kSamples;
    p.horizon_steps = kHorizon;
    p.dt = kDt;
    p.lambda = 0.1;
    p.control_cost_decoupling = 1.0;
    p.normalize_cost_spread = true;
    for (int i = 0; i < 4; ++i) {
      p.sigma.segment<3>(3 * i) << 8.0, 8.0, 15.0;   // N
      p.u_min.segment<3>(3 * i) << -60.0, -60.0, 0.0;
      p.u_max.segment<3>(3 * i) << 60.0, 60.0, 160.0;
    }
    p.seed = seed;
    return Controller(Srb{}, cost, p, SplineKnotSampler<12>(seed, 5));
  }

  // gravity-compensating stance seed: mg/4 vertical on each foot
  static Srb::Control GravitySeed(const Srb& model) {
    Srb::Control u = Srb::Control::Zero();
    const double fz = model.params().mass * model.params().gravity / 4.0;
    for (int i = 0; i < 4; ++i) u(3 * i + 2) = fz;
    return u;
  }
};

}  // namespace

TEST(MppiQuadrupedTest, StandingBalanceHoldsHeightAndAttitude) {
  SrbTrackingCost tracking;
  tracking.height_ref = kHeight;
  auto mppi = Setup::Make(tracking, AllStance(), 3);
  mppi.SeedSequence(Setup::GravitySeed(mppi.model()));

  Srb plant;  // simulate with the same model (model-mismatch tests are M4+)
  Srb::State x = Srb::MakeState(Eigen::Vector3d(0, 0, kHeight),
                                Eigen::Vector3d::Zero(),
                                Eigen::Quaterniond::Identity(),
                                Eigen::Vector3d::Zero());
  double mean_fz = 0.0;
  const int cycles = Cycles(200);  // 4 s in Release
  for (int i = 0; i < cycles; ++i) {
    mppi.model().SetContext(FeetUnderBody(Srb::Position(x)), AllStance());
    plant.SetContext(FeetUnderBody(Srb::Position(x)), AllStance());
    mppi.Plan(x);
    const auto u = mppi.Command();
    mean_fz += (u(2) + u(5) + u(8) + u(11)) / 4.0 / cycles;
    x = plant.Step(x, u, 0, kDt);
  }

  if (!kReducedScale) {
    EXPECT_NEAR(Srb::Position(x)(2), kHeight, 0.04);
    const Eigen::Vector3d body_z =
        Srb::Orientation(x) * Eigen::Vector3d::UnitZ();
    EXPECT_GT(body_z(2), std::cos(5.0 * M_PI / 180.0));  // tilt < 5 deg
    // stance forces carry the weight: mg/4 per foot on average
    const double mg4 = 15.0 * 9.81 / 4.0;
    EXPECT_NEAR(mean_fz, mg4, 0.3 * mg4);
  } else {
    (void)mean_fz;
    EXPECT_TRUE(x.allFinite());
  }
}

TEST(MppiQuadrupedTest, RecoversFromLateralPush) {
  SrbTrackingCost tracking;
  tracking.height_ref = kHeight;
  auto mppi = Setup::Make(tracking, AllStance(), 7);
  mppi.SeedSequence(Setup::GravitySeed(mppi.model()));

  Srb plant;
  Srb::State x = Srb::MakeState(
      Eigen::Vector3d(0, 0, kHeight), Eigen::Vector3d(0.0, 0.4, 0.0),
      Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
  for (int i = 0; i < Cycles(150); ++i) {  // 3 s in Release
    mppi.model().SetContext(FeetUnderBody(Srb::Position(x)), AllStance());
    plant.SetContext(FeetUnderBody(Srb::Position(x)), AllStance());
    mppi.Plan(x);
    x = plant.Step(x, mppi.Command(), 0, kDt);
  }
  if (!kReducedScale) {
    EXPECT_LT(Srb::Velocity(x).norm(), 0.10);
    EXPECT_NEAR(Srb::Position(x)(2), kHeight, 0.05);
  } else {
    EXPECT_TRUE(x.allFinite());
  }
}

TEST(MppiQuadrupedTest, TrotTracksForwardVelocity) {
  SrbTrackingCost tracking;
  tracking.height_ref = kHeight;
  tracking.velocity_ref = Eigen::Vector3d(0.3, 0.0, 0.0);
  tracking.velocity_weight = 250.0;

  const int phase_steps = 10;  // 0.2 s per diagonal pair
  auto mppi = Setup::Make(tracking, TrotSchedule(0, phase_steps), 11);
  mppi.SeedSequence(Setup::GravitySeed(mppi.model()));

  Srb plant;
  Srb::State x = Srb::MakeState(Eigen::Vector3d(0, 0, kHeight),
                                Eigen::Vector3d::Zero(),
                                Eigen::Quaterniond::Identity(),
                                Eigen::Vector3d::Zero());
  TrotGait gait(Srb::Position(x), phase_steps);
  double mean_vx = 0.0;
  int samples = 0;
  const int cycles = Cycles(300);  // 6 s in Release
  for (int i = 0; i < cycles; ++i) {
    const auto schedule = TrotSchedule(i, phase_steps);
    const auto feet_plan =
        gait.Plan(i, Srb::Position(x), tracking.velocity_ref);
    mppi.model().SetContext(feet_plan, schedule);
    plant.SetContext(feet_plan.front(), schedule);
    mppi.Plan(x);
    x = plant.Step(x, mppi.Command(), 0, kDt);
    gait.Advance(i, Srb::Position(x), tracking.velocity_ref);
    if (i >= cycles / 2) {
      mean_vx += Srb::Velocity(x)(0);
      ++samples;
    }
  }
  mean_vx /= samples;

  if (!kReducedScale) {
    EXPECT_NEAR(mean_vx, 0.3, 0.12);
    EXPECT_NEAR(Srb::Position(x)(2), kHeight, 0.05);
    EXPECT_GT(Srb::Position(x)(0), 0.6);  // actually moved forward
  } else {
    (void)mean_vx;
    EXPECT_TRUE(x.allFinite());
  }
}

TEST(MppiQuadrupedTest, FrictionConeRespectedInStance) {
  SrbTrackingCost tracking;
  tracking.height_ref = kHeight;
  auto mppi = Setup::Make(tracking, AllStance(), 13);
  mppi.SeedSequence(Setup::GravitySeed(mppi.model()));

  Srb plant;
  Srb::State x = Srb::MakeState(
      Eigen::Vector3d(0, 0, kHeight), Eigen::Vector3d(0.3, 0.0, 0.0),
      Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
  const double mu = 0.6;
  double worst_slip = 0.0;
  for (int i = 0; i < Cycles(100); ++i) {
    mppi.model().SetContext(FeetUnderBody(Srb::Position(x)), AllStance());
    plant.SetContext(FeetUnderBody(Srb::Position(x)), AllStance());
    mppi.Plan(x);
    const auto u = mppi.Command();
    for (int f = 0; f < 4; ++f) {
      EXPECT_GE(u(3 * f + 2), 0.0);  // unilateral contact (also box-clamped)
      const double slip =
          u.segment<2>(3 * f).norm() - mu * std::max(0.0, u(3 * f + 2));
      worst_slip = std::max(worst_slip, slip);
    }
    x = plant.Step(x, u, 0, kDt);
  }
  // soft constraint: small transient violations are acceptable, gross
  // violations are not (hard enforcement lives in the leg controller)
  EXPECT_LT(worst_slip, 5.0);  // N
}
