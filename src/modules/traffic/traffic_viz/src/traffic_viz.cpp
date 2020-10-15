/* 
 * traffic_viz.cpp
 * 
 * Created on: Aug 21, 2018 07:38
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "traffic_viz/traffic_viz.hpp"

#include <functional>

#include "mission/vehicle_state.hpp"

#include "decomp/curvilinear_grid_draw.hpp"
#include "state_lattice/lattice_draw.hpp"

#include "traffic_viz/roadmap_draw.hpp"
#include "traffic_viz/vehicle_draw.hpp"
#include "traffic_viz/threat_draw.hpp"
#include "traffic_viz/roadmap_viz.hpp"

using namespace ivnav;

void UGVNavViz::ShowVehicleInChannel(std::shared_ptr<RoadMap> map, Polygon polygon, TrafficChannel *channel, std::string window_name, bool save_img)
{
    auto canvas = UGVNavViz::CreateCanvas(map);
    RoadMapViz::DrawLanes(canvas, map, true);
    RoadMapViz::DrawTrafficChannelGrid(canvas, map, channel, false);
    VehicleViz::DrawVehicle(canvas, polygon);
    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void UGVNavViz::ShowVehicleStaticThreat(std::shared_ptr<RoadMap> map, VehicleStaticThreat threat, std::string window_name, bool save_img)
{
    auto canvas = UGVNavViz::CreateCanvas(map);
    VehicleFootprint fp(threat.pose);
    VehicleViz::DrawVehicleStaticCollision(canvas, threat, fp.polygon);
    RoadMapViz::DrawLanes(canvas, map, false);
    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void UGVNavViz::ShowVehicleOccupancyDistribution(std::shared_ptr<RoadMap> map, std::shared_ptr<VehicleThreat> threat, int32_t t_k, std::string window_name, bool save_img)
{
    auto canvas = UGVNavViz::CreateCanvas(map);
    for (auto &tcase : threat->possible_cases_)
    {
        if (tcase.threat_record_[t_k].occupancy_grid != nullptr)
            CurvilinearGridViz::DrawCurvilinearGridGrayscaleCost(canvas, *(tcase.threat_record_[t_k].occupancy_grid.get()));
    }
    RoadMapViz::DrawLanes(canvas, map, true);
    VehicleViz::DrawVehicle(canvas, threat->vehicle_est_.GetFootprint());

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void UGVNavViz::ShowVehicleCollisionThreat(std::shared_ptr<RoadMap> map, std::shared_ptr<VehicleThreat> threat, int32_t t_k, std::string window_name, bool save_img)
{
    auto canvas = UGVNavViz::CreateCanvas(map);

    auto center = threat->GetThreatCenter(t_k);
    ThreatViz::DrawCollisionThreat(canvas, *threat.get(), t_k, center.x, center.y, 120, 50);

    // if (threat->occupancy_grid_ != nullptr)
    //     cdraw.DrawCurvilinearGridGrayscaleCost(*(threat->occupancy_grid_.get()));

    RoadMapViz::DrawLanes(canvas, map, true);
    VehicleViz::DrawVehicle(canvas, threat->vehicle_est_.GetFootprint());

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void UGVNavViz::ShowLatticeInTrafficChannel(std::shared_ptr<RoadMap> map, std::vector<StateLattice> &lattice, TrafficChannel *channel, std::string window_name, bool save_img)
{
    auto canvas = UGVNavViz::CreateCanvas(map);

    RoadMapViz::DrawLanes(canvas, map, true);
    RoadMapViz::DrawTrafficChannelGrid(canvas, map, channel, false);

    // draw state lattice
    LatticeViz::DrawStateLattice(canvas,lattice);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void UGVNavViz::ShowLatticePathInTrafficChannel(std::shared_ptr<RoadMap> map, std::vector<StateLattice> &path, TrafficChannel *channel, std::string window_name, bool save_img)
{
    auto canvas = UGVNavViz::CreateCanvas(map);

    RoadMapViz::DrawLanes(canvas, map, true);
    RoadMapViz::DrawTrafficChannelGrid(canvas, map, channel, false);

    // draw state lattice
    LatticeViz::DrawStateLattice(canvas,path);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void UGVNavViz::ShowLatticePathInTrafficChannel(std::shared_ptr<RoadMap> map, std::shared_ptr<Graph<LatticeGraph::LatticeNode, StateLattice>> graph, std::vector<StateLattice> &path, TrafficChannel *channel, std::string window_name, bool save_img)
{
    auto canvas = UGVNavViz::CreateCanvas(map);

    RoadMapViz::DrawLanes(canvas, map, true);
    RoadMapViz::DrawTrafficChannelGrid(canvas, map, channel, false);

    std::vector<StateLattice> lattices;
    for (auto &edge : graph->GetAllEdges())
        lattices.push_back(edge->cost_);
    LatticeViz::DrawStateLattice(canvas, lattices);

    // draw state lattice
    LatticeViz::DrawStateLattice(canvas, path, 0.1, CvColors::cyan_color, 2);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

// void UGVNavViz::ShowLatticePathWithLookaheadZone(std::shared_ptr<RoadMap> map, std::vector<StateLattice> &path, LookaheadZone &zone, TrafficChannel *channel, std::string window_name, bool save_img)
// {
//     auto canvas = UGVNavViz::CreateCanvas(map);

//     RoadMapViz::DrawLanes(canvas, map, true);
//     // RoadMapViz::DrawTrafficChannelGrid(canvas, map, channel, false);
//     CurvilinearGridViz::FillCurvilinearGrid(canvas, zone, CvColors::palegreen_color);
//     // CurvilinearGridViz::DrawCurvilinearGrid(canvas, zone);

//     // draw state lattice
//     LatticeViz::DrawStateLattice(canvas, path, 0.1, CvColors::lime_color, 2);

//     CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
// }

void UGVNavViz::ShowOccupancyField(std::shared_ptr<RoadMap> map, ThreatField &field, int32_t t_k, bool show_veh_id, std::string window_name, bool save_img)
{
    auto canvas = UGVNavViz::CreateCanvas(map);

    auto threats = field.GetAllCollisionThreats();
    for (auto threat : threats)
    {
        for (auto &tcase : threat->possible_cases_)
        {
            if (tcase.intv_threat_record_[t_k].occupancy_grid != nullptr)
                CurvilinearGridViz::DrawCurvilinearGridGrayscaleCost(canvas, *(tcase.threat_record_[t_k].occupancy_grid.get()));
        }
    }

    RoadMapViz::DrawLanes(canvas, map, true);

    for (auto &veh : field.GetAllVehicleEstimations())
    {
        if (!show_veh_id)
            VehicleViz::DrawVehicle(canvas, veh.GetFootprint());
        else
            VehicleViz::DrawVehicle(canvas, veh.GetFootprint(), veh.id_);
    }

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void UGVNavViz::ShowThreatField(std::shared_ptr<RoadMap> map, ThreatField &field, int32_t t_k, bool show_veh_id, std::string window_name, bool save_img)
{
    auto canvas = UGVNavViz::CreateCanvas(map);

    auto threats = field.GetAllCollisionThreats();

    // auto center = field.GetThreatCenter(t_k);
    // tdraw.DrawCollisionThreatField(field, t_k, center.x, center.y, 160, 100);
    double xmin, xmax, ymin, ymax;
    canvas.GetCanvasRange(xmin,xmax,ymin,ymax);
    double xspan, yspan;
    canvas.GetCanvasSpan(xspan, yspan);
    double cx = (xmax + xmin) / 2.0;
    double cy = (ymax + ymin) / 2.0;
    ThreatViz::DrawCollisionThreatField(canvas, field, t_k, cx, cy, xspan, yspan);

    RoadMapViz::DrawLanes(canvas, map, true);

    for (auto &veh : field.GetAllVehicleEstimations())
    {
        if (!show_veh_id)
            VehicleViz::DrawVehicle(canvas,veh.GetFootprint());
        else
            VehicleViz::DrawVehicle(canvas,veh.GetFootprint(), veh.id_);
    }

    // draw ego vehicle
    VehicleFP ego_veh(field.ego_pose_);
    VehicleViz::DrawVehicle(canvas, ego_veh.polygon, CvColors::cyan_color);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void UGVNavViz::ShowLatticeWithOccupancyDistribution(std::shared_ptr<RoadMap> map, std::vector<StateLattice> &lattice, TrafficChannel *channel, ThreatField &field, int32_t t_k, bool show_veh_id, std::string window_name, bool save_img)
{
    auto canvas = UGVNavViz::CreateCanvas(map);

    auto threats = field.GetAllCollisionThreats();
    for (auto threat : threats)
    {
        for (auto &tcase : threat->possible_cases_)
        {
            if (tcase.intv_threat_record_[t_k].occupancy_grid != nullptr)
                CurvilinearGridViz::DrawCurvilinearGridGrayscaleCost(canvas, *(tcase.threat_record_[t_k].occupancy_grid.get()));
        }
    }

        RoadMapViz::DrawLanes(canvas, map, true);

    for (auto &veh : field.GetAllVehicleEstimations())
    {
        if (!show_veh_id)
            VehicleViz::DrawVehicle(canvas,veh.GetFootprint());
        else
            VehicleViz::DrawVehicle(canvas,veh.GetFootprint(), veh.id_);
    }

    RoadMapViz::DrawTrafficChannelGrid(canvas, map, channel, false);

    // draw state lattice
    LatticeViz::DrawStateLattice(canvas, lattice);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void UGVNavViz::ShowLatticeInThreatField(std::shared_ptr<RoadMap> map, std::vector<StateLattice> &lattice, TrafficChannel *channel, ThreatField &field, int32_t t_k, bool show_veh_id, std::string window_name, bool save_img)
{
    auto canvas = UGVNavViz::CreateCanvas(map);

    // auto threats = field.GetAllCollisionThreats();
    // for (auto threat : threats)
    // {
    //     for (auto &tcase : threat->possible_cases_)
    //     {
    //         if (tcase.intv_threat_record_[t_k].occupancy_grid != nullptr)
    //             CurvilinearGridViz::DrawCurvilinearGridGrayscaleCost(canvas, *(tcase.threat_record_[t_k].occupancy_grid.get()));
    //     }
    // }

    auto center = field.GetThreatCenter(t_k);
    ThreatViz::DrawCollisionThreatField(canvas, field, t_k, center.x, center.y, 120, 50);

    RoadMapViz::DrawLanes(canvas, map, true);

    for (auto &veh : field.GetAllVehicleEstimations())
    {
        if (!show_veh_id)
            VehicleViz::DrawVehicle(canvas,veh.GetFootprint());
        else
            VehicleViz::DrawVehicle(canvas,veh.GetFootprint(), veh.id_);
    }

    RoadMapViz::DrawTrafficChannelGrid(canvas, map, channel, false);

    // draw state lattice
    LatticeViz::DrawStateLattice(canvas, lattice);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void UGVNavViz::ShowTrafficChannelWithThreatField(std::shared_ptr<RoadMap> map, TrafficChannel *channel, ThreatField &field, int32_t t_k, bool show_veh_id, std::string window_name, bool save_img)
{
    auto canvas = UGVNavViz::CreateCanvas(map);

    // auto threats = field.GetAllCollisionThreats();
    // for (auto threat : threats)
    // {
    //     for (auto &tcase : threat->possible_cases_)
    //     {
    //         if (tcase.intv_threat_record_[t_k].occupancy_grid != nullptr)
    //             CurvilinearGridViz::DrawCurvilinearGridGrayscaleCost(canvas, *(tcase.threat_record_[t_k].occupancy_grid.get()));
    //     }
    // }

    auto center = field.GetThreatCenter(t_k);
    ThreatViz::DrawCollisionThreatField(canvas, field, t_k, center.x, center.y, 120, 50);

        RoadMapViz::DrawLanes(canvas, map, true);

    for (auto &veh : field.GetAllVehicleEstimations())
    {
        if (!show_veh_id)
            VehicleViz::DrawVehicle(canvas,veh.GetFootprint());
        else
            VehicleViz::DrawVehicle(canvas,veh.GetFootprint(), veh.id_);
    }

    RoadMapViz::DrawTrafficChannelGrid(canvas, map, channel, false);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

// void UGVNavViz::ShowPathWithThreatField(std::shared_ptr<RoadMap> map, std::vector<StateLattice> &path, LookaheadZone &zone, ThreatField &field, int32_t t_k, double T, std::string window_name, bool save_img)
// {
//     auto canvas = UGVNavViz::CreateCanvas(map);

//     double xmin, xmax, ymin, ymax;
//     canvas.GetCanvasRange(xmin,xmax,ymin,ymax);
//     double xspan, yspan;
//     canvas.GetCanvasSpan(xspan, yspan);
//     double cx = (xmax + xmin) / 2.0;
//     double cy = (ymax + ymin) / 2.0;
//     ThreatViz::DrawCollisionThreatField(canvas, field, t_k, cx, cy, xspan, yspan);

//     RoadMapViz::DrawLanes(canvas, map, true);
//     // RoadMapViz::DrawTrafficChannelGrid(canvas, map, channel, false);
//     // CurvilinearGridViz::FillCurvilinearGrid(canvas, zone, CvColors::palegreen_color);
//     // CurvilinearGridViz::DrawCurvilinearGrid(canvas, zone);

//     for (auto &veh : field.GetAllVehicleEstimations())
//         VehicleViz::DrawVehicle(canvas,veh.GetFootprint(), veh.id_);

//     // draw state lattice
//     LatticeViz::DrawStateLattice(canvas, path, 0.1, CvColors::lime_color, 2);

//     auto dstate = zone.trajectory_.GetDesiredState(t_k * T);
//     GeometryViz::DrawLabelPoint(canvas, dstate.x, dstate.y, CvColors::red_color, 2);

//     // draw ego vehicle
//     VehicleFP ego_veh({dstate.x, dstate.y, dstate.theta});
//     VehicleViz::DrawVehicle(canvas,ego_veh.polygon, CvColors::cyan_color);

//     CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
// }