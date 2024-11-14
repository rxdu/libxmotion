/*
 * @file info_planel.cpp
 * @date 11/14/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "info_planel.hpp"

namespace xmotion {
using namespace quickviz;

InfoPlanel::InfoPlanel() : Panel("Motion Panel") {
  this->SetAutoLayout(true);
  this->SetNoTitleBar(true);
  this->SetNoBackground(true);
  this->SetNoResize(true);
}

void InfoPlanel::Draw() {
  Begin();
  ImGui::Text("Frame update rate: %.3f ms/frame (%.1f FPS)",
              1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
  End();
}
}  // namespace xmotion