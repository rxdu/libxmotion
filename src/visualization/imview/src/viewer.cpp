/*
 * viewer_base.cpp
 *
 * Created on: Jul 27, 2021 08:59
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "imview/viewer.hpp"

namespace xmotion {
namespace swviz {
Viewer::Viewer(std::string title, uint32_t width, uint32_t height,
               uint32_t window_hints) {
  swviz::Init();

  window_ = std::unique_ptr<swviz::Window>(
      new swviz::Window(title, width, height, window_hints));
  window_->ApplyDarkStyle();
}

Viewer::~Viewer() { swviz::Terminate(); }

uint32_t Viewer::GetWidth() { return window_->GetWidth(); }

uint32_t Viewer::GetHeight() { return window_->GetHeight(); }

ImFont *Viewer::GetFont(FontSize size) { return window_->GetFont(size); }

void Viewer::DockSpaceOverMainViewport() {
  ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());
}

void Viewer::Show() {
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
}  // namespace swviz
}  // namespace xmotion