/*
 * @file swerve_motion_visualizer.cpp
 * @date 11/14/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "swerve_motion_visualizer.hpp"

#include "imview/box.hpp"
#include "imview/layer.hpp"

#include "info_planel.hpp"
#include "motion_drawer.hpp"

namespace xmotion {
using namespace quickviz;

xmotion::SwerveMotionVisualizer::SwerveMotionVisualizer() {
  viewer_ =
      std::make_unique<quickviz::Viewer>("Swerve Motion Visualizer", 800, 600);
  auto ui_layer = std::make_shared<quickviz::Layer>("UI Layer");
  {
    auto box = std::make_shared<quickviz::Box>("UiBox");
    auto panel = std::make_shared<InfoPlanel>();
    panel->SetFlexGrow(1.0);
    panel->SetFlexShrink(1.0);
    box->AddChild(panel);
    box->SetFlexBasis(1.0);
    ui_layer->AddChild(box);
  }

  auto cairo_layer = std::make_shared<quickviz::Layer>("Cairo Layer");
  {
    auto box = std::make_shared<quickviz::Box>("CairoBox");
    auto drawer = std::make_shared<MotionDrawer>();
    drawer->SetFlexGrow(1.0);
    drawer->SetFlexShrink(1.0);
    box->AddChild(drawer);
    box->SetFlexBasis(1.0);
    cairo_layer->AddChild(box);
  }

  viewer_->AddSceneObject(cairo_layer);
  viewer_->AddSceneObject(ui_layer);
}

bool SwerveMotionVisualizer::Initialize() { return true; }

void SwerveMotionVisualizer::Run() { viewer_->Show(); }
}  // namespace xmotion