/*
 * test_wgui.cpp
 *
 * Created on: Jul 22, 2021 14:50
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "imview/window.hpp"

using namespace robosw;

int main(int argc, char* argv[]) {
  viewer::Init();

  viewer::Window win;

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

      ImGui::PushFont(win.GetFont(viewer::FontSize::Normal));
      ImGui::Text("Canvas ");
      ImGui::PopFont();

      ImGui::PushFont(win.GetFont(viewer::FontSize::Tiny));
      ImGui::Text("Canvas ");
      ImGui::PopFont();

      ImGui::PushFont(win.GetFont(viewer::FontSize::Small));
      ImGui::Text("Canvas ");
      ImGui::PopFont();

      ImGui::PushFont(win.GetFont(viewer::FontSize::Big));
      ImGui::Text("Canvas ");
      ImGui::PopFont();

      ImGui::PushFont(win.GetFont(viewer::FontSize::Large));
      ImGui::Text("Canvas ");
      ImGui::PopFont();

      ImGui::PushFont(win.GetFont(viewer::FontSize::ExtraLarge));
      ImGui::Text("Canvas ");
      ImGui::PopFont();

      ImGui::End();
    }

    win.RenderFrame();
  }

  viewer::Terminate();
  return 0;
}