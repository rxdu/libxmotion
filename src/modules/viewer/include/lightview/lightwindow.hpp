/*
 * LightWindowdow.hpp
 *
 * Created on: May 08, 2020 09:32
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef LightWindowDOW_HPP
#define LightWindowDOW_HPP

#include <string>
#include <functional>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "lightview/lighttypes.hpp"

namespace rav {
class LightWindow {
 public:
  typedef void (*WindowKeyCallbackFunc)(GLFWwindow* window, int key,
                                        int scancode, int action, int mods);
  typedef void (*WinowErrorCallbackFunc)(int error, const char* description);
  using IterationFunc = std::function<void(void)>;

 public:
  LightWindow(int width = 640, int height = 480,
              const char* title = "LightWindow");
  virtual ~LightWindow();

  void SetBackgroundColor(float x, float y, float z, float w);
  void SetErrorCallback(WinowErrorCallbackFunc cb);
  void SetKeyCallback(WindowKeyCallbackFunc cb);

  LtColor GetBackgroundColor() const { return clear_color_; }
  void GetWindowSize(int& width, int& height);

  void SetIterativeProcedure(IterationFunc iter) { Iteration = iter; };

  virtual void Draw();
  virtual void Run();

 protected:
  GLFWwindow* window_;
  std::string glsl_version_;
  bool win_initialized_ = false;

  // window properties
  LtColor clear_color_ = {0.45f, 0.55f, 0.60f, 1.00f};

  // window behavor
  IterationFunc Iteration = nullptr;
};
}  // namespace rav

#endif /* LightWindowDOW_HPP */
