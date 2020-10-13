/*
 * lightwidgets.cpp
 *
 * Created on: Dec 07, 2018 08:04
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "lightview/lightwidgets.hpp"

#include <GL/gl3w.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace rav {
// Source: https://gist.github.com/saccadic/2a40b09e3fc4623cb14c02cac3ddbe6c
// Reference:
//  [1]
//  https://stackoverflow.com/questions/11217121/how-to-manage-memory-with-texture-in-opengl
void ViewerUtils::ConvertMatToGL(const cv::Mat &src, unsigned int *texID) {
  if (src.empty() == true) return;

  glDeleteTextures(1, texID);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glGenTextures(1, texID);
  glBindTexture(GL_TEXTURE_2D, *texID);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  if (src.channels() == 4) {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, src.cols, src.rows, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, src.ptr<unsigned char>());
  } else if (src.channels() == 3) {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, src.cols, src.rows, 0, GL_RGB,
                 GL_UNSIGNED_BYTE, src.ptr<unsigned char>());
  } else if (src.channels() == 1) {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, src.cols, src.rows, 0, GL_RED,
                 GL_UNSIGNED_BYTE, src.ptr<unsigned char>());
  } else {
    std::cerr << "convertion from cv::Mat to GL texture failed!";
  }
}

// Source: https://blog.csdn.net/weixin_43007275/article/details/82590530
void LightWidget::DrawOpenCVImageToBackground(cv::Mat img) {
  ImGuiIO &io = ImGui::GetIO();
  ImGui::SetNextWindowPos(ImVec2(0, 0), 0, ImVec2(0, 0));
  ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x, io.DisplaySize.y));
  ImGui::SetNextWindowBgAlpha(0);

  static unsigned int tex_id;
  ViewerUtils::ConvertMatToGL(img, &tex_id);

  ImTextureID bg_tex_id;
  bg_tex_id = reinterpret_cast<GLuint *>(tex_id);

  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);

  ImGui::Begin("Canvas", NULL,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |
                   ImGuiWindowFlags_NoBringToFrontOnFocus |
                   ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoCollapse |
                   ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar);
  ImGui::Image(bg_tex_id, ImGui::GetContentRegionAvail());

  ImGui::End();
  ImGui::PopStyleVar(2);
}
}  // namespace librav