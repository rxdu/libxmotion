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
#include "lightview/viewer_utils.hpp"

namespace librav
{
// Source: https://blog.csdn.net/weixin_43007275/article/details/82590530
void LightWidget::DrawOpenCVImageToBackground(cv::Mat img)
{
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

    ImGui::Begin("Canvas", NULL, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar);
    ImGui::Image(bg_tex_id, ImGui::GetContentRegionAvail());

    ImGui::End();
    ImGui::PopStyleVar(2);
}
} // namespace librav