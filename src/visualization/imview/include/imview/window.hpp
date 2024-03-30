/*
 * window.hpp
 *
 * Created on: Mar 04, 2021 15:05
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef WINDOW_HPP
#define WINDOW_HPP

#include "imgui.h"

#include "implot/implot.h"

#include <GL/gl.h>
#include <GLFW/glfw3.h>

#include <string>

namespace xmotion {
namespace swviz {
enum class FontSize { Tiny, Small, Normal, Big, Large, ExtraLarge };

void Init();
void Terminate();

class Window {
 public:
  enum WINDOW_HINT {
    WIN_FOCUSED = 0x00000001,
    WIN_RESIZABLE = 0x00000004,
    WIN_DECORATED = 0x00000010,
    WIN_AUTO_ICONIFY = 0x00000020,
    WIN_FLOATING = 0x00000040,
    WIN_MAXIMIZED = 0x00000080
  };

 public:
  Window(std::string title = "Window", uint32_t width = 640,
         uint32_t height = 480,
         uint32_t window_hints = WIN_RESIZABLE | WIN_DECORATED);
  ~Window();

  // do not allow copy
  Window(const Window &other) = delete;
  Window &operator=(const Window &other) = delete;
  Window(const Window &&other) = delete;
  Window &operator=(const Window &&other) = delete;

  void Show();

  // configuration
  void ApplyDarkStyle();
  void ApplyLightStyle();

  void EnableDocking(bool enable);
  void SetBackgroundColor(ImVec4 color);

  void EnableKeyboardNav();
  void EnableGamepadNav();

  uint32_t GetWidth() const;
  uint32_t GetHeight() const;

  // main loop
  bool WindowShouldClose();
  void CloseWindow();

  void PollEvents();
  void StartNewFrame();
  void RenderFrame();

  ImFont *GetFont(FontSize size = FontSize::Normal);
  GLFWwindow *GetGlfwWindow() { return glfw_win_; }

 private:
  GLFWwindow *glfw_win_;
  ImVec4 background_color_;

  const char *glsl_version_str = "#version 130";
  const int glfw_context_version_major = 3;
  const int glfw_context_version_minor = 0;

  ImFont *font_tiny_;
  ImFont *font_small_;
  ImFont *font_normal_;
  ImFont *font_big_;
  ImFont *font_large_;
  ImFont *font_extra_large_;

  void ApplyWindowHints(uint32_t window_hints);
  void LoadDefaultStyle();
};
}  // namespace swviz

}  // namespace xmotion

#endif /* WINDOW_HPP */
