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
ControlPanel::ControlPanel(swviz::Viewer *parent, TbotContext &ctx)
    : Panel("ControlPanel", parent), ctx_(ctx) {}

void ControlPanel::Draw() {
  Begin(NULL, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar);

  ImGui::Indent(10);

  ImGui::Dummy(ImVec2(0.0f, 5.0f));
  {
    ImGui::Text("Control Mode:");
    ImGui::SameLine();

    static int e = static_cast<int>(ctx_.control_mode);
    ImGui::RadioButton("PWM", &e, 0);
    ImGui::SameLine();
    ImGui::RadioButton("RPM", &e, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Motion", &e, 2);

    if (e != static_cast<int>(ctx_.control_mode)) {
      ctx_.control_mode = static_cast<ControlMode>(e);

      Messenger::SupervisorCommand sup_cmd;
      switch (e) {
        case 0:sup_cmd.supervised_mode = Messenger::SupervisedMode::kSupervisedPwm;
          break;
        case 1:sup_cmd.supervised_mode = Messenger::SupervisedMode::kSupervisedRpm;
          break;
        case 2:sup_cmd.supervised_mode = Messenger::SupervisedMode::kNonSupervised;
          break;
      }
      if (ctx_.msger->IsStarted()) {
        ctx_.msger->SendSupervisorCommand(sup_cmd);
      }
    }
  }

  ImGui::Dummy(ImVec2(0.0f, 5.0f));
  {
    ImGui::Text("PWM Control");
    ImGui::Separator();
    ImGui::Dummy(ImVec2(0.0f, 5.0f));

    static int pwm_left = 0, pwm_right = 0;

    ImGui::Text("Left:");
    ImGui::SameLine(75);
    ImGui::SliderInt("##slider_pwm_left", &pwm_left, -100, 100, "%d");
    ImGui::SameLine();
    if (ImGui::Button("Reset##Left-PWM")) {
      pwm_left = 0;
    }

    ImGui::Text("Right:");
    ImGui::SameLine(75);
    ImGui::SliderInt("##slider_pwm_right", &pwm_right, -100, 100, "%d");
    ImGui::SameLine();
    if (ImGui::Button("Reset##Right-PWM")) {
      pwm_right = 0;
    }

    if (ctx_.msger->IsStarted() && (ctx_.control_mode == ControlMode::kPwm)) {
      ctx_.msger->SendPwmCommand(pwm_left, pwm_right);
    }
  }

  ImGui::Dummy(ImVec2(0.0f, 5.0f));
  {
    ImGui::Text("RPM Control");
    ImGui::Separator();
    ImGui::Dummy(ImVec2(0.0f, 5.0f));

    static int rpm_left = 0, rpm_right = 0;

    ImGui::Text("Left:");
    ImGui::SameLine(75);
    ImGui::SliderInt("##slider_rpm_left", &rpm_left, -500, 500, "%d");
    ImGui::SameLine();
    if (ImGui::Button("Reset##Left-RPM")) {
      rpm_left = 0;
    }

    ImGui::Text("Right:");
    ImGui::SameLine(75);
    ImGui::SliderInt("##slider_rpm_right", &rpm_right, -500, 500, "%d");
    ImGui::SameLine();
    if (ImGui::Button("Reset##Right-RPM")) {
      rpm_right = 0;
    }

    if (ctx_.msger->IsStarted() && (ctx_.control_mode == ControlMode::kRpm)) {
      ctx_.msger->SendRpmCommand(rpm_left, rpm_right);
    }
  }

  ImGui::Dummy(ImVec2(0.0f, 5.0f));
  {
    ImGui::Text("Motion Control");
    ImGui::Separator();
    ImGui::Dummy(ImVec2(0.0f, 5.0f));

    static float linear = 0, angular = 0;

    ImGui::Text("Linear:");
    ImGui::SameLine(75);
    ImGui::SliderFloat("##slider_linear_motion", &linear, -1.5, 1.5, "%.2f");
    ImGui::SameLine();
    if (ImGui::Button("Reset##Linear-Motion")) {
      linear = 0;
    }

    ImGui::Text("Angular:");
    ImGui::SameLine(75);
    ImGui::SliderFloat("##slider_angular_motion", &angular, -1.5, 1.5, "%.2f");
    ImGui::SameLine();
    if (ImGui::Button("Reset##Angular-Motion")) {
      angular = 0;
    }

    if (ctx_.msger->IsStarted() && (ctx_.control_mode == ControlMode::kMotion)) {
      ctx_.msger->SendMotionCommand(linear, angular);
    }
  }

  ImGui::Unindent(10);

  End();
}
}  // namespace robosw