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

void TrafficViz::ShowLatticePathInTrafficChannel(std::vector<StateLattice> &lattice, TrafficChannel &channel, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(false);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    LatticeDraw lattice_draw = LatticeDraw(canvas);

    road_draw.DrawLanes(false);
    // road_draw.DrawTrafficChannelGrid(channel, false);

    // draw state lattice
    lattice_draw.DrawStateLattice(lattice);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowVehicleStaticThreat(VehicleStaticThreat threat, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(true);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);

    veh_draw.DrawVehicleStaticCollision(threat, threat.footprint.polygon);
    road_draw.DrawLanes(false);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowVehicleOccupancyDistribution(std::shared_ptr<CollisionThreat> threat, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(false);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);
    CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);

    if (threat->occupancy_grid_ != nullptr)
        cdraw.DrawCurvilinearGridGrayscaleCost(*(threat->occupancy_grid_.get()));

    road_draw.DrawLanes(true);
    veh_draw.DrawVehicle(threat->vehicle_est_.GetFootprint());

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowVehicleIntervalOccupancyDistribution(std::shared_ptr<CollisionThreat> threat, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(false);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);
    CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);

    if (threat->interval_occupancy_grid_ != nullptr)
        cdraw.DrawCurvilinearGridGrayscaleCost(*(threat->interval_occupancy_grid_.get()));

    road_draw.DrawLanes(true);
    veh_draw.DrawVehicle(threat->vehicle_est_.GetFootprint());

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowVehicleOccupancyDistribution(std::vector<std::shared_ptr<CollisionThreat>> threats, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(false);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);
    CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);

    for (auto threat : threats)
    {
        if (threat->occupancy_grid_ != nullptr)
            cdraw.DrawCurvilinearGridGrayscaleCost(*(threat->occupancy_grid_.get()));
    }

    road_draw.DrawLanes(true);

    for (auto threat : threats)
        veh_draw.DrawVehicle(threat->vehicle_est_.GetFootprint());

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowVehicleCollisionThreat(std::shared_ptr<CollisionThreat> threat, int32_t t_k, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(true);

    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);
    // CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);
    GeometryDraw gdraw = GeometryDraw(canvas);

    auto center = threat->GetThreatCenter();
    gdraw.DrawDistribution(center.x, center.y, 100, 50,
                           std::bind(*threat.get(), std::placeholders::_1, std::placeholders::_2, t_k));

    // if (threat->occupancy_grid_ != nullptr)
    //     cdraw.DrawCurvilinearGridGrayscaleCost(*(threat->occupancy_grid_.get()));

    road_draw.DrawLanes(true);
    veh_draw.DrawVehicle(threat->vehicle_est_.GetFootprint());

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowVehicleIntervalCollisionThreat(std::shared_ptr<CollisionThreat> threat, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(true);

    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);
    // CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);
    GeometryDraw gdraw = GeometryDraw(canvas);

    auto center = threat->GetThreatCenter();
    gdraw.DrawDistribution(center.x, center.y, 100, 50, std::bind(*threat.get(), std::placeholders::_1, std::placeholders::_2, true));

    // if (threat->occupancy_grid_ != nullptr)
    //     cdraw.DrawCurvilinearGridGrayscaleCost(*(threat->occupancy_grid_.get()));

    road_draw.DrawLanes(true);
    veh_draw.DrawVehicle(threat->vehicle_est_.GetFootprint());

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowOccupancyField(ThreatField &field, bool show_veh_id, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(false);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);
    CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);

    auto threats = field.GetAllCollisionThreats();

    for (auto threat : threats)
    {
        if (threat->occupancy_grid_ != nullptr)
            cdraw.DrawCurvilinearGridGrayscaleCost(*(threat->occupancy_grid_.get()));
    }

    road_draw.DrawLanes(true);

    for (auto threat : threats)
    {
        if (!show_veh_id)
            veh_draw.DrawVehicle(threat->vehicle_est_.GetFootprint());
        else
            veh_draw.DrawVehicle(threat->vehicle_est_.GetFootprint(), threat->vehicle_est_.id_);
    }

    ShowImage(canvas.paint_area, window_name, save_img);
}

void TrafficViz::ShowThreatField(ThreatField &field, bool show_veh_id, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(true);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);
    CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);
    GeometryDraw gdraw = GeometryDraw(canvas);

    auto threats = field.GetAllCollisionThreats();

    auto center = field.GetThreatCenter();
    gdraw.DrawDistribution(center.x, center.y, 150, 120, field);

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
    GeometryDraw gdraw = GeometryDraw(canvas);

    auto threats = field.GetAllCollisionThreats();

    auto center = field.GetThreatCenter(t_k);
    gdraw.DrawDistribution(center.x, center.y, 150, 120, std::bind(field, std::placeholders::_1, std::placeholders::_2, t_k));

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

void TrafficViz::ShowLatticeWithOccupancyDistribution(std::vector<StateLattice> &lattice, TrafficChannel &channel, ThreatField &field, bool show_veh_id, std::string window_name, bool save_img)
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
        if (threat->occupancy_grid_ != nullptr)
            cdraw.DrawCurvilinearGridGrayscaleCost(*(threat->occupancy_grid_.get()));
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

void TrafficViz::ShowLatticeInThreatField(std::vector<StateLattice> &lattice, TrafficChannel &channel, ThreatField &field, bool show_veh_id, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(true);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);
    CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);
    LatticeDraw lattice_draw = LatticeDraw(canvas);
    GeometryDraw gdraw = GeometryDraw(canvas);

    // auto threats = field.GetAllCollisionThreats();
    // for (auto threat : threats)
    // {
    //     if (threat->occupancy_grid_ != nullptr)
    //         cdraw.DrawCurvilinearGridGrayscaleCost(*(threat->occupancy_grid_.get()));
    // }
    auto center = field.GetThreatCenter();
    gdraw.DrawDistribution(center.x, center.y, 150, 120, field);

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

void TrafficViz::ShowTrafficChannelWithThreatField(TrafficChannel &channel, ThreatField &field, bool show_veh_id, std::string window_name, bool save_img)
{
    RoadMapViz &viz = RoadMapViz::GetInstance();

    CartesianCanvas canvas = viz.CreateCanvas(true);
    RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
    VehicleDraw veh_draw = VehicleDraw(canvas);
    CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);
    LatticeDraw lattice_draw = LatticeDraw(canvas);
    GeometryDraw gdraw = GeometryDraw(canvas);

    // auto threats = field.GetAllCollisionThreats();
    // for (auto threat : threats)
    // {
    //     if (threat->occupancy_grid_ != nullptr)
    //         cdraw.DrawCurvilinearGridGrayscaleCost(*(threat->occupancy_grid_.get()));
    // }
    auto center = field.GetThreatCenter();
    gdraw.DrawDistribution(center.x, center.y, 150, 120, field);

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