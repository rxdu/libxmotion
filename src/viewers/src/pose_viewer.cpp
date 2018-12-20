/* 
 * pose_viewer.cpp
 * 
 * Created on: Dec 19, 2018 07:55
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "viewers/pose_viewer.hpp"

#include <cassert>

#include "lightview/viewer_utils.hpp"
#include "lightview/lightwidgets.hpp"
#include "lightviz/details/cartesian_canvas.hpp"

using namespace librav;

PoseViewer::PoseViewer() : LightViewer()
{
    // setup communication link
    data_link_ = std::make_shared<LCMLink>();
    if (!data_link_->good())
        std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
    data_link_ready_ = true;

    // data_link_->subscribe(CAV_COMMON_CHANNELS::VEHICLE_ESTIMATIONS_CHANNEL, &PoseViewer::HandleVehicleEstimationsMsg, this);
    // data_link_->subscribe(CAV_COMMON_CHANNELS::EGO_VEHICLE_STATE_CHANNEL, &PoseViewer::HandleEgoVehicleStateMsg, this);
}

void PoseViewer::HandleEgoVehicleStateMsg(const librav::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::VehicleState *msg)
{
    // ego_vehicle_state_ = VehicleState(msg->id, {msg->position[0], msg->position[1], msg->theta}, msg->speed);
    // ego_state_updated_ = true;
}

void PoseViewer::HandleVehicleEstimationsMsg(const librav::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::VehicleEstimations *msg)
{
    // surrounding_vehicles_.clear();
    // for (int i = 0; i < msg->vehicle_num; ++i)
    // {
    //     auto est = msg->estimations[i];
    //     surrounding_vehicles_.emplace_back(VehicleState(est.id, {est.position[0], est.position[1], est.theta}, est.speed));
    // }
}

void PoseViewer::Start()
{
    std::cout << "traffic viewer started" << std::endl;

    bool show_another_window = false;

    while (!glfwWindowShouldClose(window_))
    {
        PreHouseKeeping();

        // process LCM callbacks first
        data_link_->handleTimeout(0);

        // create image according to user selection
        //-------------------------------------------------------------//

        static float f = 0.0f;
        static int counter = 0;

        static bool show_center_line = false;
        static bool use_jetcolor_bg = false;
        static bool centered_at_ego_vehicle = true;
        static float zoom_in_ratio = 0.56f;
        static bool save_image = false;

        //-------------------------------------------------------------//

        ImGui::SetNextWindowPos(ImVec2(0, 0), 0, ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(100, 700), ImGuiCond_FirstUseEver);

        ImGui::Begin("Settings:");

        ImGui::Text("Road Map:");
        ImGui::Checkbox("Show Centerline", &show_center_line);
        ImGui::Checkbox("Jetcolor Map", &use_jetcolor_bg);

        ImGui::Text("Traffic:");
        ImGui::Checkbox("Centered at Ego Vehicle", &centered_at_ego_vehicle);
        ImGui::SliderFloat("Zoom-in Ratio", &zoom_in_ratio, 0.0f, 1.0f);
        ImGui::Text("Save Result:");
        static char img_name_buf[32] = "traffic_viewer";
        ImGui::InputText("", img_name_buf, IM_ARRAYSIZE(img_name_buf));
        ImGui::SameLine();
        if (ImGui::Button("Save Image"))
            save_image = true;
        // ImGui::SameLine();

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

        ImGui::End();

        //-------------------------------------------------------------//

        // CartesianCanvas canvas(ppu_);
        // if (use_jetcolor_bg)
        //     canvas.SetupCanvas(xmin_, xmax_, ymin_, ymax_, CvDrawColors::jet_colormap_lowest);
        // else
        //     canvas.SetupCanvas(xmin_, xmax_, ymin_, ymax_);

        // RoadMapDraw road_draw(road_map_, canvas);
        // road_draw.DrawLanes(show_center_line);

        // VehicleDraw veh_draw(canvas);
        // for (auto &veh : surrounding_vehicles_)
        // {
        //     Polygon fp = VehicleFootprint(veh.GetPose()).polygon;
        //     veh_draw.DrawVehicle(fp, veh.id_);
        // }

        // if (ego_state_updated_)
        // {
        //     Polygon fp = VehicleFootprint(ego_vehicle_state_.GetPose()).polygon;
        //     veh_draw.DrawVehicle(fp, CvDrawColors::blue_color);
        // }

        //-------------------------------------------------------------//

        // draw image to window
        cv::Mat vis_img = cv::imread("/home/rdu/Pictures/lanelets_light.png");
        // cv::Mat vis_img = canvas.paint_area;
        // cv::Mat vis_img = canvas.GetROIofPaintArea(80, 100, 100, 100);

        // cv::Mat vis_img;
        // if (centered_at_ego_vehicle && ego_state_updated_)
        // {
        //     auto pos = ego_vehicle_state_.GetPose().position;
        //     vis_img = canvas.GetROIofPaintArea(pos.x, pos.y, zoom_in_ratio);
        // }
        // else
        // {
        //     vis_img = canvas.paint_area;
        // }

        LightWidget::DrawOpenCVImageToBackground(vis_img);

        // save image if requested
        if (save_image)
        {
            std::string name(img_name_buf);
            cv::imwrite(name + ".png", vis_img);
            save_image = false;
        }

        vis_img.release();

        //-------------------------------------------------------------//

        PostHousekeeping();
    }
}