/* 
 * control_panel.cpp
 *
 * Created on 4/3/22 11:12 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "tbot/control_panel.hpp"

namespace robosw {
ControlPanel::ControlPanel(swviz::Viewer *parent, TbotContext &ctx) :
    Panel("ControlPanel", parent), ctx_(ctx) {

}

void ControlPanel::Draw() {
  Begin(NULL, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar);

  ImGui::Indent(10);

  bool can_ready = ((ctx_.can_ != nullptr) && ctx_.can_->IsOpened());

  ImGui::Dummy(ImVec2(0.0f, 5.0f));
  {
    ImGui::Text("PWM Control");
    ImGui::Separator();
    ImGui::Dummy(ImVec2(0.0f, 5.0f));

    static int pwm_left = 0, pwm_right = 0;

    ImGui::Text("Left:");
    ImGui::SameLine(60);
    ImGui::SliderInt("##slider_pwm_left", &pwm_left, -100, 100, "%d");
    ImGui::SameLine();
    if (ImGui::Button("Reset##Left")) { pwm_left = 0; }

    ImGui::Text("Right:");
    ImGui::SameLine(60);
    ImGui::SliderInt("##slider_pwm_right", &pwm_right, -100, 100, "%d");
    ImGui::SameLine();
    if (ImGui::Button("Reset##Right")) { pwm_right = 0; }

    if (can_ready) {
      struct can_frame frame;
      frame.can_id = 0x101;
      frame.can_dlc = 2;
      frame.data[0] = static_cast<uint8_t>(pwm_left);
      frame.data[1] = static_cast<uint8_t>(pwm_right);
      ctx_.can_->SendFrame(frame);
    }
  }

  ImGui::Dummy(ImVec2(0.0f, 5.0f));
  {
    ImGui::Text("Motor Control");
    ImGui::Separator();
    ImGui::Dummy(ImVec2(0.0f, 5.0f));
  }

  ImGui::Unindent(10);

  End();
}
}