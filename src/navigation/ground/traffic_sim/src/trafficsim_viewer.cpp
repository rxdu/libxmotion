/* 
 * trafficsim_viewer.cpp
 * 
 * Created on: Feb 17, 2019 03:34
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "trafficsim_viewer/trafficsim_viewer.hpp"

#include <cassert>

#include "cvdraw/cvdraw.hpp"
#include "navviz/roadmap_draw.hpp"
#include "navviz/vehicle_draw.hpp"

#include "traffic_map/map_loader.hpp"
#include "cav_common/cav_datalink.hpp"

using namespace librav;

TrafficSimViewer::TrafficSimViewer(std::string map_file, int32_t ppu) : ppu_(ppu)
{
    MapLoader loader(map_file);
    road_map_ = loader.road_map;
    traffic_map_ = loader.traffic_map;

    CalcCanvasSize(road_map_);

    // setup communication link
    data_link_ = std::make_shared<LCMLink>();
    if (!data_link_->good())
        std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
    data_link_ready_ = true;

    data_link_->subscribe(CAV_COMMON_CHANNELS::VEHICLE_ESTIMATIONS_CHANNEL, &TrafficSimViewer::HandleVehicleEstimationsMsg, this);
    data_link_->subscribe(CAV_COMMON_CHANNELS::EGO_VEHICLE_STATE_CHANNEL, &TrafficSimViewer::HandleEgoVehicleStateMsg, this);
}

void TrafficSimViewer::CalcCanvasSize(std::shared_ptr<RoadMap> map)
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

    x_span_ = xmax_ - xmin_;
    y_span_ = ymax_ - ymin_;
    aspect_ratio_ = y_span_ / x_span_;
}

cv::Mat TrafficSimViewer::CropImageToROI(cv::Mat img, double cx, double cy, double xspan, double yspan)
{
    double rxmin = cx - xspan / 2.0;
    double rxmax = cx + xspan / 2.0;
    double rymin = cy - yspan / 2.0;
    double rymax = cy + yspan / 2.0;

    if (rxmin < xmin_)
        rxmin = xmin_;
    if (rxmax > xmax_)
        rxmax = xmax_;
    if (rymin < ymin_)
        rymin = ymin_;
    if (rymax > ymax_)
        rymax = ymax_;

    std::cout << "canvas size: " << xmin_ << " , " << xmax_ << " , " << ymin_ << " , " << ymax_ << std::endl;

    int tl_x = rxmin * ppu_;
    int tl_y = rymin * ppu_;
    int width = (rxmax - rxmin) * ppu_;
    int height = (rymax - rymin) * ppu_;

    return img(cv::Rect(tl_x, tl_y, width, height));
}

void TrafficSimViewer::HandleEgoVehicleStateMsg(const librav::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::VehicleState *msg)
{
    ego_vehicle_state_ = VehicleState(msg->id, {msg->position[0], msg->position[1], msg->theta}, msg->speed);
    ego_state_updated_ = true;
}

void TrafficSimViewer::HandleVehicleEstimationsMsg(const librav::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::VehicleEstimations *msg)
{
    surrounding_vehicles_.clear();
    for (int i = 0; i < msg->vehicle_num; ++i)
    {
        auto est = msg->estimations[i];
        surrounding_vehicles_.emplace_back(VehicleState(est.id, {est.position[0], est.position[1], est.theta}, est.speed));
    }
}

void TrafficSimViewer::Start()
{
    std::cout << "traffic sim viewer started" << std::endl;

    bool show_another_window = false;
    bool use_jetcolor_bg = true;
    bool show_center_line = true;
    bool centered_at_ego_vehicle = false;
    bool save_image = false;

    double zoom_in_ratio = 0.5;

    while (true)
    {
        loop_stopwatch_.tic();

        // process LCM callbacks first
        data_link_->handleTimeout(0);

        CvCanvas canvas(ppu_);
        canvas.Resize(xmin_, xmax_, ymin_, ymax_);
        if (use_jetcolor_bg)
            canvas.FillBackgroundColor(CvColors::jet_colormap_lowest);

        RoadMapViz::DrawLanes(canvas, road_map_, show_center_line);

        for (auto &veh : surrounding_vehicles_)
        {
            Polygon fp = VehicleFootprint(veh.GetPose()).polygon;
            VehicleViz::DrawVehicle(canvas, fp, veh.id_);
        }

        if (ego_state_updated_)
        {
            Polygon fp = VehicleFootprint(ego_vehicle_state_.GetPose()).polygon;
            VehicleViz::DrawVehicle(canvas, fp, CvColors::blue_color);
        }

        //-------------------------------------------------------------//

        // draw image to window
        // cv::Mat vis_img = cv::imread("/home/rdu/Pictures/lanelets_light.png");

        cv::Mat vis_img;
        if (centered_at_ego_vehicle && ego_state_updated_)
        {
            auto pos = ego_vehicle_state_.GetPose().position;
            vis_img = canvas.GetROIofPaintArea(pos.x, pos.y, zoom_in_ratio);
        }
        else
        {
            vis_img = canvas.GetPaintArea();
        }

        // LightWidget::DrawOpenCVImageToBackground(vis_img);

        // save image if requested
        if (save_image)
        {
            std::string name = "sim_default";
            cv::imwrite(name + ".png", vis_img);
            save_image = false;
        }

        // show frame
        CvIO::ShowImageFrame(vis_img, "Traffic Sim");

        //-------------------------------------------------------------//

        loop_stopwatch_.sleep_until_ms(1000 / 60);
    }
}