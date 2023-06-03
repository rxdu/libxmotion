/*
 * test_wgui.cpp
 *
 * Created on: Jul 22, 2021 14:50
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "imview/window.hpp"

using namespace xmotion;

int main(int argc, char* argv[]) {
  swviz::Init();

  swviz::Window win;

  win.ApplyDarkStyle();

  while (!win.WindowShouldClose()) {
    // handle events
    win.PollEvents();
    if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Escape))) {
      win.CloseWindow();
    }

    // draw stuff
    win.StartNewFrame();

    {
      ImGui::Begin("Canvas ");
      //   ImGui::Begin("Canvas ", NULL,
      //                ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize
      //                |
      //                    ImGuiWindowFlags_NoMove);

      ImGui::PushFont(win.GetFont(swviz::FontSize::Normal));
      ImGui::Text("Canvas ");
      ImGui::PopFont();

      ImGui::PushFont(win.GetFont(swviz::FontSize::Tiny));
      ImGui::Text("Canvas ");
      ImGui::PopFont();

      ImGui::PushFont(win.GetFont(swviz::FontSize::Small));
      ImGui::Text("Canvas ");
      ImGui::PopFont();

      ImGui::PushFont(win.GetFont(swviz::FontSize::Big));
      ImGui::Text("Canvas ");
      ImGui::PopFont();

      ImGui::PushFont(win.GetFont(swviz::FontSize::Large));
      ImGui::Text("Canvas ");
      ImGui::PopFont();

      ImGui::PushFont(win.GetFont(swviz::FontSize::ExtraLarge));
      ImGui::Text("Canvas ");
      ImGui::PopFont();

      ImGui::End();
    }

    win.RenderFrame();
  }

  swviz::Terminate();
  return 0;
}