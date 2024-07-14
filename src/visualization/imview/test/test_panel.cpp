/*
 * test_viewer.cpp
 *
 * Created on: Jul 27, 2021 09:07
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "imview/viewer.hpp"
#include "imview/panel.hpp"

using namespace xmotion::swviz;

class PanelFixed : public Panel {
 public:
  PanelFixed(std::string name, Viewer *parent) : Panel(name, parent) {}

  void Draw() {
    auto work_pos = ImGui::GetMainViewport()->WorkPos;
    auto work_size = ImGui::GetMainViewport()->WorkSize;
    ImGui::SetNextWindowPos(
        ImVec2(work_pos.x, work_pos.y + work_size.y * 1 / 3));
    ImGui::SetNextWindowSize(ImVec2(work_size.x, work_size.y * 2 / 3));
    Begin(NULL, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar |
                    ImGuiWindowFlags_NoDocking |
                    ImGuiWindowFlags_NoBringToFrontOnFocus);

    ImGui::SetCursorPos(ImVec2(180, 200));
    ImGui::PushFont(parent_->GetFont(FontSize::Normal));
    ImGui::Text("Test Panel");
    ImGui::PopFont();
    ImVec2 size = ImGui::GetWindowSize();
    printf("PanelNoDock: %f x %f\n", size.x, size.y);
    if (ImGui::IsWindowFocused()) {
      ImGui::SetCursorPos(ImVec2(300, 500));
      if (ImGui::Button("IsWindowFocused", ImVec2(200, 50))) {
        printf("IsWindowFocused button clicked.\n");
      }
    }
    if (ImGui::IsWindowHovered()) {
      ImGui::SetCursorPos(ImVec2(800, 500));
      if (ImGui::Button("IsWindowHovered", ImVec2(200, 50))) {
        printf("IsWindowHovered button clicked.\n");
      }
    }
    End();
  }
};

class PanelDockable : public Panel {
 public:
  PanelDockable(std::string name, Viewer *parent) : Panel(name, parent) {}

  void Draw() {
    Begin(NULL, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar);
    ImGui::SetCursorPos(ImVec2(10, 5));
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 0, 0, 200));
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::PopStyleColor();
    ImVec2 size = ImGui::GetWindowSize();
    printf("PanelDockable: %f x %f\n", size.x, size.y);
    if (ImGui::IsWindowFocused()) {
      ImGui::SetCursorPos(ImVec2(300, 500));
      if (ImGui::Button("IsWindowFocused", ImVec2(200, 50))) {
        printf("IsWindowFocused button clicked.\n");
      }
    }
    if (ImGui::IsWindowHovered()) {
      ImGui::SetCursorPos(ImVec2(800, 500));
      if (ImGui::Button("IsWindowHovered", ImVec2(200, 50))) {
        printf("IsWindowHovered button clicked.\n");
      }
    }
    End();
  }
};

class DockingRegion : public Panel {
 public:
  DockingRegion(std::string name, Viewer *parent) : Panel(name, parent) {}

  void Draw() {
    SetupDockSpace();
    auto work_pos = ImGui::GetMainViewport()->WorkPos;
    auto work_size = ImGui::GetMainViewport()->WorkSize;
    ImGui::SetNextWindowPos(ImVec2(work_pos));
    ImGui::SetNextWindowSize(ImVec2(work_size.x, work_size.y * 1 / 3));
    Begin(NULL, ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoTitleBar |
                    ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar |
                    ImGuiWindowFlags_NoResize |
                    ImGuiWindowFlags_NoBringToFrontOnFocus |
                    ImGuiWindowFlags_NoNavFocus);
    if (ImGui::BeginMainMenuBar()) {
      if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("Exit", "Esc")) printf("EXIT\n");
        ImGui::EndMenu();
      }

      if (ImGui::BeginMenu("Settings")) {
        if (ImGui::MenuItem("Edit", "Ctrl+N")) printf("EDIT\n");

        if (ImGui::MenuItem("Restore Defaults", "Ctrl+R")) printf("EDIT\n");

        ImGui::EndMenu();
      }
      ImGui::EndMainMenuBar();
    }
    End();
  }

 private:
  void SetupDockSpace() {
    static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_None;
    ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
    ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);
  }
};

class MyViewer : public Viewer {
 public:
  MyViewer() : Viewer("MyViewer", 2000, 1500) {}
  void Draw() {
    panel_non_dockable_.Draw();
    docking_region_.Draw();
    panel_dockable_a.Draw();
    panel_dockable_b.Draw();
  }

  PanelFixed panel_non_dockable_{"FIXED PANEL", this};
  DockingRegion docking_region_{"Docking Region", this};
  PanelDockable panel_dockable_a{"Panel: DOCKABLE_A", this};
  PanelDockable panel_dockable_b{"Panel: DOCKABLE_B", this};
};

int main(int argc, char *argv[]) {
  MyViewer viewer;
  viewer.Show();
  return 0;
}