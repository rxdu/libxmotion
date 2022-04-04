/* 
 * canbus_panel.cpp
 *
 * Created on 4/4/22 9:56 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "tbot/canbus_panel.hpp"

#include <memory>

namespace robosw {
CanbusPanel::CanbusPanel(swviz::Viewer *parent, TbotContext &ctx) :
    Panel("CanbusPanel", parent), ctx_(ctx) {

}

void CanbusPanel::Draw() {
  Begin(NULL, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar);
  ImGui::Indent(10);

  ImGui::Dummy(ImVec2(0.0f, 5.0f));
  {
    const char *item_names[] = {"can0", "can1", "vcan0", "vcan1"};
    static int port_idx = 0;
    static bool item_disabled = false;

    ImGui::SetCursorPos(ImVec2(10, 40));
    ImGui::Text("Port:");
//    ImGui::SameLine(60);
    ImGui::SetCursorPos(ImVec2(80, 38));
    ImGui::SetNextItemWidth(100);
    ImGui::Combo("##CAN_Port", &port_idx, item_names, IM_ARRAYSIZE(item_names), IM_ARRAYSIZE(item_names));
//    ImGui::SameLine(200);
    ImGui::SetCursorPos(ImVec2(220, 38));
    if (ctx_.can_ == nullptr || !ctx_.can_->IsOpened()) {
      if (ImGui::Button("Connect")) {
        if (ctx_.can_ != nullptr) {
          ctx_.can_.reset();
        }
        std::string port = {item_names[port_idx]};
        ctx_.can_ = std::make_shared<AsyncCAN>(port);
        ctx_.can_->Open();
      }
    } else {
      if (ImGui::Button("Disconnect")) {
        ctx_.can_->Close();
      }
    }
  }

  ImGui::Unindent(10);
  End();
}
}