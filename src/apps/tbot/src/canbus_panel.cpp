/* 
 * canbus_panel.cpp
 *
 * Created on 4/4/22 9:56 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "tbot/canbus_panel.hpp"
#include "imview/popup.hpp"

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
    ImGui::SetCursorPos(ImVec2(80, 38));
    ImGui::SetNextItemWidth(100);
    ImGui::Combo("##CAN_Port", &port_idx, item_names,
                 IM_ARRAYSIZE(item_names), IM_ARRAYSIZE(item_names));
    ImGui::SetCursorPos(ImVec2(220, 38));
    static bool show_open_error_popup = false;
    if (!ctx_.msger->IsStarted()) {
      if (ImGui::Button("Connect")) {
        std::string port = {item_names[port_idx]};
        show_open_error_popup = !(ctx_.msger->Start(port));
      }
    } else {
      if (ImGui::Button("Disconnect")) {
        ctx_.msger->Stop();
      }
    }
    if (show_open_error_popup) {
      show_open_error_popup = swviz::ShowPopupNotification("Failed to open port", "Notification");
    }
  }

  ImGui::Unindent(10);
  End();
}
}