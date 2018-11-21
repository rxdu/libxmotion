/* 
 * traffic_viewer.cpp
 * 
 * Created on: Nov 21, 2018 01:07
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "ugv_viz/traffic_viewer.hpp"

#include <cassert>

#include "lightview/viewer_utils.hpp"
#include "lightviz/details/cartesian_canvas.hpp"
#include "ugvnav_viz/details/roadmap_draw.hpp"

#include "traffic_map/map_loader.hpp"

using namespace librav;

TrafficViewer::TrafficViewer(std::string map_file, int32_t ppu) : ppu_(ppu)
{
    MapLoader loader(map_file);
    road_map_ = loader.road_map;
    traffic_map_ = loader.traffic_map;

    CalcCanvasSize(road_map_);
}

void TrafficViewer::CalcCanvasSize(std::shared_ptr<RoadMap> map)
{
    assert(map != nullptr);

    std::vector<double> xmins, xmaxs, ymins, ymaxs;

    for (auto &polyline : map->GetAllLaneBoundPolylines())
    {
        xmins.push_back(polyline.GetMinX());
        xmaxs.push_back(polyline.GetMaxX());
        ymins.push_back(polyline.GetMinY());
        ymaxs.push_back(polyline.GetMaxY());
    }
    for (auto &polyline : map->GetAllLaneCenterPolylines())
    {
        xmins.push_back(polyline.GetMinX());
        xmaxs.push_back(polyline.GetMaxX());
        ymins.push_back(polyline.GetMinY());
        ymaxs.push_back(polyline.GetMaxY());
    }
    double bd_xl = *std::min_element(xmins.begin(), xmins.end());
    double bd_xr = *std::max_element(xmaxs.begin(), xmaxs.end());
    double bd_yb = *std::min_element(ymins.begin(), ymins.end());
    double bd_yt = *std::max_element(ymaxs.begin(), ymaxs.end());

    double xspan = bd_xr - bd_xl;
    double yspan = bd_yt - bd_yb;

    xmin_ = bd_xl - xspan * 0.1;
    xmax_ = bd_xr + xspan * 0.1;
    ymin_ = bd_yb - yspan * 0.1;
    ymax_ = bd_yt + yspan * 0.1;
}

// Source: https://blog.csdn.net/weixin_43007275/article/details/82590530
void TrafficViewer::DrawOpenCVImage(cv::Mat img)
{
    ImGuiIO &io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(0, 0), 0, ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x, io.DisplaySize.y));
    ImGui::SetNextWindowBgAlpha(0);

    ImTextureID bg_tex_id;
    // bg_tex_id = (GLuint *)loadTexture(cvLoadImage("Template/bg2.jpg"));
    unsigned int tex_id;
    ViewerUtils::ConvertMatToGL(img, &tex_id);
    bg_tex_id = reinterpret_cast<GLuint *>(tex_id);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);

    ImGui::Begin("Canvas", NULL, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar);
    ImGui::Image(bg_tex_id, ImGui::GetContentRegionAvail());

    ImGui::End();
    ImGui::PopStyleVar(2);
}

void TrafficViewer::Start()
{
    std::cout << "traffic viewer started" << std::endl;

    bool show_another_window = false;

    while (!glfwWindowShouldClose(window_))
    {
        PreHouseKeeping();

        // create image according to user selection
        //-------------------------------------------------------------//

        cv::Mat vis_img;

        static float f = 0.0f;
        static int counter = 0;

        static bool show_center_line = true;
        static bool use_jetcolor_bg = false;
        static bool save_image = false;

        //-------------------------------------------------------------//

        // ImGui::SetNextWindowPos(ImVec2(0, 0), 0, ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(100, 720), ImGuiCond_FirstUseEver);

        ImGui::Begin("Hello, world!");

        ImGui::Text("This is some useful text.");
        ImGui::Checkbox("Show Centerline", &show_center_line);
        ImGui::Checkbox("Jetcolor Map", &use_jetcolor_bg);

        static char img_name_buf[32] = "traffic_viewer";
        ImGui::InputText("", img_name_buf, IM_ARRAYSIZE(img_name_buf));
        ImGui::SameLine();
        if (ImGui::Button("Save Image"))
            save_image = true;
        // ImGui::SameLine();

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

        ImGui::End();

        //-------------------------------------------------------------//

        // vis_img = cv::imread("/home/rdu/Pictures/lanelets_light.png");

        CartesianCanvas canvas(ppu_);
        if (use_jetcolor_bg)
            canvas.SetupCanvas(xmin_, xmax_, ymin_, ymax_, CvDrawColors::jet_colormap_lowest);
        else
            canvas.SetupCanvas(xmin_, xmax_, ymin_, ymax_);

        RoadMapDraw road_draw = RoadMapDraw(road_map_, canvas);
        road_draw.DrawLanes(show_center_line);

        vis_img = canvas.paint_area;

        // draw image to window
        DrawOpenCVImage(vis_img);

        // save image if requested
        if (save_image)
        {
            std::string name(img_name_buf);
            cv::imwrite(name + ".png", vis_img);
            save_image = false;
        }

        //-------------------------------------------------------------//

        PostHousekeeping();
    }
}