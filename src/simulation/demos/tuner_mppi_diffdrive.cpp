/*
 * tuner_mppi_diffdrive.cpp
 *
 * Interactive MPPI tuner (Viz v2): a differential-drive robot runs against
 * the demo obstacle course on a background thread while the quickviz viewer
 * shows, live:
 *   - the world panel (CairoWidget): obstacle course, executed trail, and
 *     the candidate-rollout fan with alpha-blended softmax weights
 *   - ESS and best-cost strip charts (RtLinePlotWidget via BufferRegistry)
 *   - a control panel: pause / single-step / reset and live lambda / sigma
 *     sliders applied to the controller between Plan() calls
 *
 * Threading follows the quickviz contract: all GL/ImGui work stays on the
 * main thread; the simulation publishes frames through a latest-only
 * DataStream and plot points through registry ring buffers; tuning knobs
 * travel UI -> sim through atomics and are applied on the planning thread.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <atomic>
#include <chrono>
#include <cstdio>
#include <deque>
#include <memory>
#include <thread>
#include <utility>

#include "core/buffer/buffer_registry.hpp"
#include "core/buffer/ring_buffer.hpp"
#include "core/data_stream.hpp"
#include "canvas/cairo_widget.hpp"
#include "plot/rt_line_plot_widget.hpp"
#include "viewer/box.hpp"
#include "viewer/viewer.hpp"

#include "xmnav/mppi/critics.hpp"
#include "xmnav/models/diff_drive.hpp"
#include "xmnav/mppi/mppi.hpp"
#include "xmnav/viz/mppi_draw_cairo.hpp"

using namespace xmotion;

namespace {

using Cost = decltype(MakeCompositeCost(std::declval<Se2GoalCost>(),
                                        std::declval<CircularObstacleCost>()));
using Controller = Mppi<DiffDriveModel, Cost>;
using PlotPoint = quickviz::RtLinePlotWidget::DataPoint;

constexpr int kNumSamples = 1024;
constexpr double kDt = 0.05;

// UI -> sim knobs and sim -> UI gauges (all lock-free)
struct TunerShared {
  std::atomic<double> lambda{0.3};
  std::atomic<double> sigma_v{0.3};
  std::atomic<double> sigma_w{0.8};
  std::atomic<bool> paused{false};
  std::atomic<int> step_credits{0};
  std::atomic<bool> reset{false};
  std::atomic<bool> stop{false};
  std::atomic<double> ess{0.0};
  std::atomic<double> best_cost{0.0};
  std::atomic<double> sim_time{0.0};
};

// one sim step as seen by the world panel
struct WorldFrame {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double t = 0.0;
  Controller::Snapshot snapshot;
};

void RunSim(TunerShared &shared, quickviz::DataStream<WorldFrame> &stream,
            const Se2GoalCost &goal_cost,
            const CircularObstacleCost &obstacle_cost) {
  auto &registry = quickviz::BufferRegistry::GetInstance();
  auto ess_buffer = *registry.GetBuffer<PlotPoint>("mppi.ess");
  auto cost_buffer = *registry.GetBuffer<PlotPoint>("mppi.best_cost");

  Controller::Params p;
  p.num_samples = kNumSamples;
  p.horizon_steps = 50;
  p.dt = kDt;
  p.lambda = shared.lambda.load();
  p.sigma << shared.sigma_v.load(), shared.sigma_w.load();
  p.u_min << -0.8, -2.0;
  p.u_max << 0.8, 2.0;
  p.normalize_cost_spread = true;
  Controller mppi(DiffDriveModel{},
                  MakeCompositeCost(goal_cost, obstacle_cost), p);
  mppi.EnableIntrospection(24, 24);

  DiffDriveModel plant;
  DiffDriveModel::State x = DiffDriveModel::State::Zero();
  double t = 0.0;

  while (!shared.stop.load()) {
    if (shared.reset.exchange(false)) {
      x.setZero();
      t = 0.0;
      mppi.Reset();
    }
    if (shared.paused.load()) {
      int credits = shared.step_credits.load();
      if (credits <= 0 ||
          !shared.step_credits.compare_exchange_strong(credits, credits - 1)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
    }

    // apply UI knobs on the planning thread, between Plan() calls
    mppi.SetTemperature(shared.lambda.load());
    Controller::Control sigma;
    sigma << shared.sigma_v.load(), shared.sigma_w.load();
    mppi.SetSigma(sigma);

    mppi.Plan(x);
    x = plant.Step(x, mppi.Command(), 0, kDt);
    t += kDt;

    WorldFrame frame;
    frame.x = x(0);
    frame.y = x(1);
    frame.theta = x(2);
    frame.t = t;
    frame.snapshot = mppi.LastSnapshot();
    stream.Push(std::move(frame));

    const float tf = static_cast<float>(t);
    ess_buffer->Write({tf, static_cast<float>(mppi.LastEffectiveSampleSize())});
    cost_buffer->Write({tf, static_cast<float>(mppi.LastBestCost())});
    shared.ess.store(mppi.LastEffectiveSampleSize());
    shared.best_cost.store(mppi.LastBestCost());
    shared.sim_time.store(t);

    // real-time pacing; while paused the single-step path above still
    // executes exactly one iteration per credit
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(kDt * 1000)));
  }
}

class TunerControlPanel : public quickviz::Panel {
 public:
  explicit TunerControlPanel(TunerShared &shared)
      : Panel("mppi_tuner_controls"), shared_(shared) {
    SetAutoLayout(true);
    SetNoResize(true);
    SetNoMove(true);
    SetWindowNoMenuButton();
  }

  void Draw() override {
    Begin();
    ImGui::Text("t = %6.1f s   ESS = %5.0f / %d   best cost = %.2f",
                shared_.sim_time.load(), shared_.ess.load(), kNumSamples,
                shared_.best_cost.load());
    ImGui::Separator();

    float lambda = static_cast<float>(shared_.lambda.load());
    if (ImGui::SliderFloat("lambda (temperature)", &lambda, 0.02f, 2.0f,
                           "%.3f", ImGuiSliderFlags_Logarithmic)) {
      shared_.lambda.store(lambda);
    }
    float sigma_v = static_cast<float>(shared_.sigma_v.load());
    if (ImGui::SliderFloat("sigma v [m/s]", &sigma_v, 0.02f, 1.0f, "%.3f")) {
      shared_.sigma_v.store(sigma_v);
    }
    float sigma_w = static_cast<float>(shared_.sigma_w.load());
    if (ImGui::SliderFloat("sigma omega [rad/s]", &sigma_w, 0.05f, 3.0f,
                           "%.3f")) {
      shared_.sigma_w.store(sigma_w);
    }
    ImGui::Separator();

    const bool paused = shared_.paused.load();
    if (ImGui::Button(paused ? "Resume" : "Pause")) {
      shared_.paused.store(!paused);
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!paused);
    if (ImGui::Button("Step")) {
      shared_.step_credits.fetch_add(1);
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
      shared_.reset.store(true);
    }
    End();
  }

 private:
  TunerShared &shared_;
};

}  // namespace

int main() {
  Se2GoalCost goal_cost;
  goal_cost.goal << 3.5, 1.0, 0.0;
  CircularObstacleCost obstacle_cost;
  obstacle_cost.obstacles.push_back({Eigen::Vector2d(1.5, 0.2), 0.4});
  obstacle_cost.obstacles.push_back({Eigen::Vector2d(2.6, 1.1), 0.3});

  // plot buffers must exist before AddLine() resolves them by name
  auto &registry = quickviz::BufferRegistry::GetInstance();
  registry.AddBuffer<PlotPoint>(
      "mppi.ess", std::make_shared<quickviz::RingBuffer<PlotPoint, 1024>>());
  registry.AddBuffer<PlotPoint>(
      "mppi.best_cost",
      std::make_shared<quickviz::RingBuffer<PlotPoint, 1024>>());

  TunerShared shared;
  quickviz::DataStream<WorldFrame> stream;
  std::thread sim(RunSim, std::ref(shared), std::ref(stream),
                  std::cref(goal_cost), std::cref(obstacle_cost));

  quickviz::Viewer viewer("xmnav MPPI tuner", 1600, 900);

  // world panel: drain the stream and render the latest frame (all on the
  // render thread; `latest`/`trail` are touched by the draw func only)
  auto world = std::make_shared<quickviz::CairoWidget>("world", true);
  world->OnResize(960, 640);  // initial surface; yoga resizes on first frame
  world->SetAutoLayout(true);
  world->SetNoResize(true);
  world->SetNoMove(true);
  auto latest = std::make_shared<WorldFrame>();
  auto trail = std::make_shared<std::deque<Eigen::Vector2d>>();
  CairoWorldFrame view;
  view.x_min = -0.5;
  view.x_max = 4.5;
  view.y_min = -1.0;
  view.y_max = 2.0;
  world->AttachDrawFunction([&stream, latest, trail, view, goal_cost,
                             obstacle_cost](cairo_t *cr, float aspect) {
    WorldFrame frame;
    if (stream.TryPull(frame)) {
      if (frame.t < latest->t) trail->clear();  // sim was reset
      *latest = std::move(frame);
      trail->push_back(Eigen::Vector2d(latest->x, latest->y));
      if (trail->size() > 2000) trail->pop_front();
    }
    cairo_set_source_rgba(cr, viz_style::kCairoBackground.x,
                          viz_style::kCairoBackground.y,
                          viz_style::kCairoBackground.z, 1.0);
    cairo_paint(cr);
    for (const auto &ob : obstacle_cost.obstacles) {
      DrawDiscCairo(cr, aspect, view, ob.center, ob.radius,
                    viz_style::kCairoObstacle);
    }
    DrawDiscCairo(cr, aspect, view, goal_cost.goal.head<2>(), 0.06,
                  viz_style::kCairoGoal);
    DrawTrailCairo(cr, aspect, view, *trail);
    DrawMppiSnapshotCairo(cr, aspect, view, latest->snapshot);
    DrawRobotCairo(cr, aspect, view, latest->x, latest->y, latest->theta,
                   0.12);
  });

  auto ess_plot = std::make_shared<quickviz::RtLinePlotWidget>("ess_plot");
  ess_plot->SetAutoLayout(true);
  ess_plot->SetAxisLabels("t", "ESS");
  ess_plot->SetAxisUnits("s", "");
  ess_plot->SetFixedHistory(20.0f);
  ess_plot->SetYAxisRange(0.0f, static_cast<float>(kNumSamples));
  ess_plot->AddLine("ESS", "mppi.ess");

  auto cost_plot = std::make_shared<quickviz::RtLinePlotWidget>("cost_plot");
  cost_plot->SetAutoLayout(true);
  cost_plot->SetAxisLabels("t", "best cost");
  cost_plot->SetAxisUnits("s", "");
  cost_plot->SetFixedHistory(20.0f);
  cost_plot->SetYAxisRange(0.0f, 200.0f);
  cost_plot->AddLine("best cost", "mppi.best_cost");

  auto controls = std::make_shared<TunerControlPanel>(shared);

  auto plots = std::make_shared<quickviz::Box>("plots");
  plots->SetFlexDirection(quickviz::Styling::FlexDirection::kColumn);
  plots->SetAlignItems(quickviz::Styling::AlignItems::kStretch);
  plots->SetFlexGrow(1);
  plots->SetFlexShrink(1);
  ess_plot->SetFlexGrow(1);
  ess_plot->SetFlexShrink(1);
  cost_plot->SetFlexGrow(1);
  cost_plot->SetFlexShrink(1);
  plots->AddChild(ess_plot);
  plots->AddChild(cost_plot);

  auto main_row = std::make_shared<quickviz::Box>("main_row");
  main_row->SetFlexDirection(quickviz::Styling::FlexDirection::kRow);
  main_row->SetAlignItems(quickviz::Styling::AlignItems::kStretch);
  main_row->SetFlexGrow(1);
  main_row->SetFlexShrink(1);
  world->SetFlexGrow(2);
  world->SetFlexShrink(1);
  main_row->AddChild(world);
  main_row->AddChild(plots);

  auto root = std::make_shared<quickviz::Box>("root");
  root->SetFlexDirection(quickviz::Styling::FlexDirection::kColumn);
  root->SetAlignItems(quickviz::Styling::AlignItems::kStretch);
  controls->SetHeight(180);
  controls->SetFlexGrow(0);
  controls->SetFlexShrink(0);
  root->AddChild(main_row);
  root->AddChild(controls);

  viewer.AddSceneObject(root);
  viewer.Show();

  shared.stop.store(true);
  sim.join();
  return 0;
}
