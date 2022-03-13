/*
 * viewer_base.cpp
 *
 * Created on: Jul 27, 2021 08:59
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "imview/viewer_base.hpp"

namespace robosw {
namespace viewer {
ViewerBase::ViewerBase(uint32_t width, uint32_t height, std::string title,
                       uint32_t window_hints) {
  viewer::Init();

  window_ = std::unique_ptr<viewer::Window>(
      new viewer::Window(width, height, title, window_hints));
  window_->ApplyDarkStyle();
}

ViewerBase::~ViewerBase() { viewer::Terminate(); }

void ViewerBase::Show() {
  while (!window_->WindowShouldClose()) {
    // handle events
    window_->PollEvents();
    if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Escape))) {
      window_->CloseWindow();
    }

    // draw stuff
    window_->StartNewFrame();

    Update();

    window_->RenderFrame();
  }
}
}  // namespace viewer
}  // namespace robosw