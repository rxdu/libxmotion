/* 
 * popup.cpp
 *
 * Created on 4/5/22 11:08 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "imview/popup.hpp"

#include "imgui.h"

namespace xmotion {
bool swviz::ShowPopupNotification(std::string msg, std::string title,
                                  float width, float height) {
  bool show_popup = true;

  ImGui::OpenPopup(title.c_str());

  // Always center this window when appearing
  ImVec2 center = ImGui::GetMainViewport()->GetCenter();
  ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
  ImGui::SetNextWindowSize(ImVec2(width, height));
  ImGui::SetNextWindowBgAlpha(0.75f);

  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 0));

  if (ImGui::BeginPopupModal(
      title.c_str(), NULL,
      ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoResize)) {
    ImGui::SetCursorPos(ImVec2(15, 40));
    ImGui::Text("%s", msg.c_str());
    ImGui::Text("\n");
    // ImGui::Separator();

    ImGui::SetItemDefaultFocus();
    ImGui::SetCursorPos(ImVec2((width - 120) / 2.0f, 100));
    if (ImGui::Button("OK", ImVec2(120, 0))) {
      ImGui::CloseCurrentPopup();
      show_popup = false;
    }
    ImGui::EndPopup();
  }

  ImGui::PopStyleVar(1);

  return show_popup;
}
}