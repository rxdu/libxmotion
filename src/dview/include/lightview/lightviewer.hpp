/* 
 * lightviewer.hpp
 * 
 * Created on: Nov 15, 2018 10:39
 * Description: a viewer wrapper adapted from imgui example.
 *              This class provides a few convenience functions
 *              to help you set up a window easily.
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LIGHTVIEWER_HPP
#define LIGHTVIEWER_HPP

#include <string>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

// #include <GL/gl3w.h>
// #include <GL/gl.h>
// #include <GL/glew.h>
// #include <GL/glu.h>// 

#include <glad/glad.h>
// #include <GLFW/glfw3.h>

// Include glfw3.h after OpenGL definitions
#include <GLFW/glfw3.h>

namespace autodrive
{
class LightViewer
{
  public:
    LightViewer() = default;
    virtual ~LightViewer();

    // copy is not allowed
    LightViewer(const LightViewer &other) = delete;
    LightViewer &operator=(const LightViewer &other) = delete;

    // viewer setup
    bool SetupViewer(int width = 1280, int hight = 720, std::string title = "Lightviewer", bool dark_color = true);
    void SetClearColor(ImVec4 cclor) { clear_color_ = cclor; }

    // viewer main loop
    virtual void Start();

  protected:
    GLFWwindow *window_;
    ImVec4 clear_color_;

    void PreHouseKeeping();
    void PostHousekeeping();
    // Note: PostHousekeeping() = PrepareOpenGLDraw() + RenderData()
    void PrepareOpenGLDraw(int &display_w, int &display_h);
    void RenderData();
};
} // namespace autodrive

#endif /* LIGHTVIEWER_HPP */
