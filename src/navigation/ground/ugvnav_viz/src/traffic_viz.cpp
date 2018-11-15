/* 
 * traffic_viz.cpp
 * 
 * Created on: Aug 21, 2018 07:38
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "ugvnav_viz/traffic_viz.hpp"

#include <functional>

#include "lightviz/details/geometry_draw.hpp"
#include "ugvnav_viz/details/roadmap_draw.hpp"
#include "ugvnav_viz/details/vehicle_draw.hpp"
#include "ugvnav_viz/details/lattice_draw.hpp"
#include "ugvnav_viz/details/threat_draw.hpp"
#include "lightviz/details/curvilinear_grid_draw.hpp"
#include "ugvnav_viz/roadmap_viz.hpp"

using namespace librav;
using namespace CvDraw;

void TrafficViz::SetupTrafficViz(std::shared_ptr<RoadMap> map, int32_t pixel_per_unit)
{
    RoadMapViz::SetupRoadMapViz(map, pixel_per_unit);
}

void TrafficViz::ShowLanes(bool show_center_line, std::string window_name, bool save_img)
{
    RoadMapViz::ShowLanes(show_center_line, window_name, save_img);
}

void TrafficViz::ShowTrafficChannel(TrafficChannel &channel, std::string window_name, bool save_img)
{
    RoadMapViz::ShowTrafficChannel(channel, window_name, save_img);
}

void TrafficViz::ShowVehicle(Polygon polygon, std::string window_name, bool save_img)
{
    RoadMapViz::ShowVehicle(polygon, window_name, save_img);
}

void TrafficViz::ShowVehicle(std::vector<Polygon> &polygons, std::string window_name, bool save_img)
{
    RoadMapViz::ShowVehicle(polygons, window_name, save_img);
}

void TrafficViz::ShowVehicleInChannel(Polygon polygon, TrafficChannel &channel, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas();
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);

    road_draw.DrawLanes(true);
    road_draw.DrawTrafficChannelGrid(channel, false);
    veh_draw.DrawVehicle(polygon);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowLatticeInTrafficChannel(std::vector<StateLattice> &lattice, TrafficChannel &channel, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(false);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    LatticeDraw lattice_draw = LatticeDraw(canvas);

    road_draw.DrawLanes(false);
    road_draw.DrawTrafficChannelGrid(channel, false);

    // draw state lattice
    lattice_draw.DrawStateLattice(lattice);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowLatticePathInTrafficChannel(std::vector<StateLattice> &path, TrafficChannel &channel, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(false);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    LatticeDraw lattice_draw = LatticeDraw(canvas);

    road_draw.DrawLanes(false);
    // road_draw.DrawTrafficChannelGrid(channel, false);

    // draw state lattice
    lattice_draw.DrawStateLattice(path);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowLatticePathInTrafficChannel(std::shared_ptr<Graph<LatticeGraph::LatticeNode, StateLattice>> graph, std::vector<StateLattice> &path, TrafficChannel &channel, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(false);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    LatticeDraw lattice_draw = LatticeDraw(canvas);

    road_draw.DrawLanes(false);
    road_draw.DrawTrafficChannelGrid(channel, false);

    std::vector<StateLattice> lattices;
    for (auto &edge : graph->GetAllEdges())
        lattices.push_back(edge->cost_);
    lattice_draw.DrawStateLattice(lattices);

    // draw state lattice
    lattice_draw.DrawStateLattice(path, 0.1, CvDrawColors::cyan_color, 2);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowLatticePathWithLookaheadZone(std::vector<StateLattice> &path, LookaheadZone &zone, TrafficChannel &channel, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(false);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    LatticeDraw lattice_draw = LatticeDraw(canvas);
    CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);

    road_draw.DrawLanes(false);
    // road_draw.DrawTrafficChannelGrid(channel, false);
    cdraw.DrawFilledCurvilinearGrid(zone, CvDrawColors::palegreen_color);
    // cdraw.DrawCurvilinearGrid(zone);

    // draw state lattice
    lattice_draw.DrawStateLattice(path, 0.1, CvDrawColors::lime_color, 2);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowVehicleStaticThreat(VehicleStaticThreat threat, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(true);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);

    // veh_draw.DrawVehicleStaticCollision(threat, threat.footprint.polygon);
    road_draw.DrawLanes(false);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowVehicleOccupancyDistribution(std::shared_ptr<VehicleThreat> threat, int32_t t_k, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(false);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);
    CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);

    for (auto &tcase : threat->possible_cases_)
    {
        if (tcase.threat_record_[t_k].occupancy_grid != nullptr)
            cdraw.DrawCurvilinearGridGrayscaleCost(*(tcase.threat_record_[t_k].occupancy_grid.get()));
    }

    road_draw.DrawLanes(true);
    veh_draw.DrawVehicle(threat->vehicle_est_.GetFootprint());

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowVehicleIntervalOccupancyDistribution(std::shared_ptr<VehicleThreat> threat, int32_t t_k, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(false);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);
    CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);

    for (auto &tcase : threat->possible_cases_)
    {
        if (tcase.intv_threat_record_[t_k].occupancy_grid != nullptr)
            cdraw.DrawCurvilinearGridGrayscaleCost(*(tcase.threat_record_[t_k].occupancy_grid.get()));
    }

    road_draw.DrawLanes(true);
    veh_draw.DrawVehicle(threat->vehicle_est_.GetFootprint());

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowVehicleCollisionThreat(std::shared_ptr<VehicleThreat> threat, int32_t t_k, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(true);

    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);
    // CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);
    ThreatDraw tdraw = ThreatDraw(canvas);

    auto center = threat->GetThreatCenter(t_k);
    tdraw.DrawCollisionThreat(*threat.get(), t_k, center.x, center.y, 120, 50);

    // if (threat->occupancy_grid_ != nullptr)
    //     cdraw.DrawCurvilinearGridGrayscaleCost(*(threat->occupancy_grid_.get()));

    road_draw.DrawLanes(true);
    veh_draw.DrawVehicle(threat->vehicle_est_.GetFootprint());

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowVehicleIntervalCollisionThreat(std::shared_ptr<VehicleThreat> threat, int32_t t_k, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(true);

    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);
    // CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);
    ThreatDraw tdraw = ThreatDraw(canvas);

    auto center = threat->GetThreatCenter(t_k);
    tdraw.DrawCollisionThreat(*threat.get(), t_k, center.x, center.y, 120, 50);

    // if (threat->occupancy_grid_ != nullptr)
    //     cdraw.DrawCurvilinearGridGrayscaleCost(*(threat->occupancy_grid_.get()));

    road_draw.DrawLanes(true);
    veh_draw.DrawVehicle(threat->vehicle_est_.GetFootprint());

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowOccupancyField(ThreatField &field, int32_t t_k, bool show_veh_id, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(false);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);
    CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);

    auto threats = field.GetAllCollisionThreats();

    for (auto threat : threats)
    {
        for (auto &tcase : threat->possible_cases_)
        {
            if (tcase.intv_threat_record_[t_k].occupancy_grid != nullptr)
                cdraw.DrawCurvilinearGridGrayscaleCost(*(tcase.threat_record_[t_k].occupancy_grid.get()));
        }
    }

    road_draw.DrawLanes(true);

    for (auto &veh : field.GetAllVehicleEstimations())
    {
        if (!show_veh_id)
            veh_draw.DrawVehicle(veh.GetFootprint());
        else
            veh_draw.DrawVehicle(veh.GetFootprint(), veh.id_);
    }

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowThreatField(ThreatField &field, int32_t t_k, bool show_veh_id, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(true);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);
    CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);
    ThreatDraw tdraw = ThreatDraw(canvas);

    auto threats = field.GetAllCollisionThreats();

    auto center = field.GetThreatCenter(t_k);
    tdraw.DrawCollisionThreatField(field, t_k, center.x, center.y, 160, 100);

    road_draw.DrawLanes(true);

    for (auto &veh : field.GetAllVehicleEstimations())
    {
        if (!show_veh_id)
            veh_draw.DrawVehicle(veh.GetFootprint());
        else
            veh_draw.DrawVehicle(veh.GetFootprint(), veh.id_);
    }

    // draw ego vehicle
    VehicleFP ego_veh(field.ego_pose_);
    veh_draw.DrawVehicle(ego_veh.polygon, CvDrawColors::cyan_color);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowLatticeWithOccupancyDistribution(std::vector<StateLattice> &lattice, TrafficChannel &channel, ThreatField &field, int32_t t_k, bool show_veh_id, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(false);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);
    CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);
    LatticeDraw lattice_draw = LatticeDraw(canvas);

    auto threats = field.GetAllCollisionThreats();

    for (auto threat : threats)
    {
        for (auto &tcase : threat->possible_cases_)
        {
            if (tcase.intv_threat_record_[t_k].occupancy_grid != nullptr)
                cdraw.DrawCurvilinearGridGrayscaleCost(*(tcase.threat_record_[t_k].occupancy_grid.get()));
        }
    }

    road_draw.DrawLanes(true);

    for (auto &veh : field.GetAllVehicleEstimations())
    {
        if (!show_veh_id)
            veh_draw.DrawVehicle(veh.GetFootprint());
        else
            veh_draw.DrawVehicle(veh.GetFootprint(), veh.id_);
    }

    road_draw.DrawTrafficChannelGrid(channel, false);

    // draw state lattice
    lattice_draw.DrawStateLattice(lattice);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowLatticeInThreatField(std::vector<StateLattice> &lattice, TrafficChannel &channel, ThreatField &field, int32_t t_k, bool show_veh_id, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(true);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);
    CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);
    LatticeDraw lattice_draw = LatticeDraw(canvas);
    ThreatDraw tdraw = ThreatDraw(canvas);

    // auto threats = field.GetAllCollisionThreats();
    // for (auto threat : threats)
    // {
    //     for (auto &tcase : threat->possible_cases_)
    //     {
    //         if (tcase.intv_threat_record_[t_k].occupancy_grid != nullptr)
    //             cdraw.DrawCurvilinearGridGrayscaleCost(*(tcase.threat_record_[t_k].occupancy_grid.get()));
    //     }
    // }

    auto center = field.GetThreatCenter(t_k);
    tdraw.DrawCollisionThreatField(field, t_k, center.x, center.y, 120, 50);

    road_draw.DrawLanes(true);

    for (auto &veh : field.GetAllVehicleEstimations())
    {
        if (!show_veh_id)
            veh_draw.DrawVehicle(veh.GetFootprint());
        else
            veh_draw.DrawVehicle(veh.GetFootprint(), veh.id_);
    }

    road_draw.DrawTrafficChannelGrid(channel, false);

    // draw state lattice
    lattice_draw.DrawStateLattice(lattice);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowTrafficChannelWithThreatField(TrafficChannel &channel, ThreatField &field, int32_t t_k, bool show_veh_id, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(true);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);
    CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);
    LatticeDraw lattice_draw = LatticeDraw(canvas);
    ThreatDraw tdraw = ThreatDraw(canvas);

    // auto threats = field.GetAllCollisionThreats();
    // for (auto threat : threats)
    // {
    //     for (auto &tcase : threat->possible_cases_)
    //     {
    //         if (tcase.intv_threat_record_[t_k].occupancy_grid != nullptr)
    //             cdraw.DrawCurvilinearGridGrayscaleCost(*(tcase.threat_record_[t_k].occupancy_grid.get()));
    //     }
    // }

    auto center = field.GetThreatCenter(t_k);
    tdraw.DrawCollisionThreatField(field, t_k, center.x, center.y, 120, 50);

    road_draw.DrawLanes(true);

    for (auto &veh : field.GetAllVehicleEstimations())
    {
        if (!show_veh_id)
            veh_draw.DrawVehicle(veh.GetFootprint());
        else
            veh_draw.DrawVehicle(veh.GetFootprint(), veh.id_);
    }

    road_draw.DrawTrafficChannelGrid(channel, false);

    ShowImage(canvas.paint_area, window_name, save_img);
}
