/* 
 * traffic_viz.cpp
 * 
 * Created on: Aug 21, 2018 07:38
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "lightviz/traffic_viz.hpp"

#include <functional>

#include "cav_common/vehicle_state.hpp"
#include "coreviz/coreviz.hpp"

#include "navviz/roadmap_draw.hpp"
#include "navviz/vehicle_draw.hpp"
#include "navviz/lattice_draw.hpp"
#include "navviz/threat_draw.hpp"
#include "lightviz/roadmap_viz.hpp"

using namespace librav;

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

void UGVNavViz::ShowVehicleIntervalOccupancyDistribution(std::shared_ptr<RoadMap> map, std::shared_ptr<VehicleThreat> threat, int32_t t_k, std::string window_name, bool save_img)
{
    auto canvas = UGVNavViz::CreateCanvas(map);
    for (auto &tcase : threat->possible_cases_)
    {
        if (tcase.intv_threat_record_[t_k].occupancy_grid != nullptr)
            CurvilinearGridViz::DrawCurvilinearGridGrayscaleCost(canvas, *(tcase.threat_record_[t_k].occupancy_grid.get()));
    }
    RoadMapViz::DrawLanes(canvas, map, true);
    VehicleViz::DrawVehicle(canvas, threat->vehicle_est_.GetFootprint());

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

// void UGVNavViz::ShowLatticePathInTrafficChannel(std::vector<StateLattice> &path, TrafficChannel *channel, std::string window_name, bool save_img)
// {
//     RoadMapViz &viz = RoadMapViz::GetInstance();

//     CartesianCanvas canvas = viz.CreateCanvas(false);
//     RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
//     LatticeDraw lattice_draw = LatticeDraw(canvas);

//     road_draw.DrawLanes(false);
//     // road_draw.DrawTrafficChannelGrid(channel, false);

//     // draw state lattice
//     lattice_draw.DrawStateLattice(path);

//     CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
// }

// void UGVNavViz::ShowLatticePathInTrafficChannel(std::shared_ptr<Graph<LatticeGraph::LatticeNode, StateLattice>> graph, std::vector<StateLattice> &path, TrafficChannel *channel, std::string window_name, bool save_img)
// {
//     RoadMapViz &viz = RoadMapViz::GetInstance();

//     CartesianCanvas canvas = viz.CreateCanvas(false);
//     RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
//     LatticeDraw lattice_draw = LatticeDraw(canvas);

//     road_draw.DrawLanes(false);
//     road_draw.DrawTrafficChannelGrid(channel, false);

//     std::vector<StateLattice> lattices;
//     for (auto &edge : graph->GetAllEdges())
//         lattices.push_back(edge->cost_);
//     lattice_draw.DrawStateLattice(lattices);

//     // draw state lattice
//     lattice_draw.DrawStateLattice(path, 0.1, CvDrawColors::cyan_color, 2);

//     CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
// }

// void TrafficViz::ShowLatticePathWithLookaheadZone(std::vector<StateLattice> &path, LookaheadZone &zone, TrafficChannel *channel, std::string window_name, bool save_img)
// {
//     RoadMapViz &viz = RoadMapViz::GetInstance();

//     CartesianCanvas canvas = viz.CreateCanvas(false);
//     RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
//     LatticeDraw lattice_draw = LatticeDraw(canvas);
//     CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);

//     road_draw.DrawLanes(false);
//     // road_draw.DrawTrafficChannelGrid(channel, false);
//     cdraw.DrawFilledCurvilinearGrid(zone, CvDrawColors::palegreen_color);
//     // cdraw.DrawCurvilinearGrid(zone);

//     // draw state lattice
//     lattice_draw.DrawStateLattice(path, 0.1, CvDrawColors::lime_color, 2);

//     CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
// }

// void TrafficViz::ShowVehicleCollisionThreat(std::shared_ptr<VehicleThreat> threat, int32_t t_k, std::string window_name, bool save_img)
// {
//     RoadMapViz &viz = RoadMapViz::GetInstance();

//     CartesianCanvas canvas = viz.CreateCanvas(true);

//     RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
//     VehicleViz veh_draw = VehicleViz(canvas);
//     // CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);
//     ThreatDraw tdraw = ThreatDraw(canvas);

//     auto center = threat->GetThreatCenter(t_k);
//     tdraw.DrawCollisionThreat(*threat.get(), t_k, center.x, center.y, 120, 50);

//     // if (threat->occupancy_grid_ != nullptr)
//     //     cdraw.DrawCurvilinearGridGrayscaleCost(*(threat->occupancy_grid_.get()));

//     road_draw.DrawLanes(true);
//     veh_draw.DrawVehicle(threat->vehicle_est_.GetFootprint());

//     CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
// }

// void TrafficViz::ShowVehicleIntervalCollisionThreat(std::shared_ptr<VehicleThreat> threat, int32_t t_k, std::string window_name, bool save_img)
// {
//     RoadMapViz &viz = RoadMapViz::GetInstance();

//     CartesianCanvas canvas = viz.CreateCanvas(true);

//     RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
//     VehicleViz veh_draw = VehicleViz(canvas);
//     // CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);
//     ThreatDraw tdraw = ThreatDraw(canvas);

//     auto center = threat->GetThreatCenter(t_k);
//     tdraw.DrawCollisionThreat(*threat.get(), t_k, center.x, center.y, 120, 50);

//     // if (threat->occupancy_grid_ != nullptr)
//     //     cdraw.DrawCurvilinearGridGrayscaleCost(*(threat->occupancy_grid_.get()));

//     road_draw.DrawLanes(true);
//     veh_draw.DrawVehicle(threat->vehicle_est_.GetFootprint());

//     CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
// }

// void TrafficViz::ShowOccupancyField(ThreatField &field, int32_t t_k, bool show_veh_id, std::string window_name, bool save_img)
// {
//     RoadMapViz &viz = RoadMapViz::GetInstance();

//     CartesianCanvas canvas = viz.CreateCanvas(false);
//     RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
//     VehicleViz veh_draw = VehicleViz(canvas);
//     CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);

//     auto threats = field.GetAllCollisionThreats();

//     for (auto threat : threats)
//     {
//         for (auto &tcase : threat->possible_cases_)
//         {
//             if (tcase.intv_threat_record_[t_k].occupancy_grid != nullptr)
//                 cdraw.DrawCurvilinearGridGrayscaleCost(*(tcase.threat_record_[t_k].occupancy_grid.get()));
//         }
//     }

//     road_draw.DrawLanes(true);

//     for (auto &veh : field.GetAllVehicleEstimations())
//     {
//         if (!show_veh_id)
//             veh_draw.DrawVehicle(veh.GetFootprint());
//         else
//             veh_draw.DrawVehicle(veh.GetFootprint(), veh.id_);
//     }

//     CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
// }

// void TrafficViz::ShowThreatField(ThreatField &field, int32_t t_k, bool show_veh_id, std::string window_name, bool save_img)
// {
//     RoadMapViz &viz = RoadMapViz::GetInstance();

//     CartesianCanvas canvas = viz.CreateCanvas(true);
//     RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
//     VehicleViz veh_draw = VehicleViz(canvas);
//     CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);
//     ThreatDraw tdraw = ThreatDraw(canvas);

//     auto threats = field.GetAllCollisionThreats();

//     // auto center = field.GetThreatCenter(t_k);
//     // tdraw.DrawCollisionThreatField(field, t_k, center.x, center.y, 160, 100);
//     double cx = (canvas.xmax_ + canvas.xmin_) / 2.0;
//     double cy = (canvas.ymax_ + canvas.ymin_) / 2.0;
//     tdraw.DrawCollisionThreatField(field, t_k, cx, cy, canvas.xspan_, canvas.yspan_);

//     road_draw.DrawLanes(true);

//     for (auto &veh : field.GetAllVehicleEstimations())
//     {
//         if (!show_veh_id)
//             veh_draw.DrawVehicle(veh.GetFootprint());
//         else
//             veh_draw.DrawVehicle(veh.GetFootprint(), veh.id_);
//     }

//     // draw ego vehicle
//     VehicleFP ego_veh(field.ego_pose_);
//     veh_draw.DrawVehicle(ego_veh.polygon, CvDrawColors::cyan_color);

//     CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
// }

// void TrafficViz::ShowLatticeWithOccupancyDistribution(std::vector<StateLattice> &lattice, TrafficChannel *channel, ThreatField &field, int32_t t_k, bool show_veh_id, std::string window_name, bool save_img)
// {
//     RoadMapViz &viz = RoadMapViz::GetInstance();

//     CartesianCanvas canvas = viz.CreateCanvas(false);
//     RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
//     VehicleViz veh_draw = VehicleViz(canvas);
//     CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);
//     LatticeDraw lattice_draw = LatticeDraw(canvas);

//     auto threats = field.GetAllCollisionThreats();

//     for (auto threat : threats)
//     {
//         for (auto &tcase : threat->possible_cases_)
//         {
//             if (tcase.intv_threat_record_[t_k].occupancy_grid != nullptr)
//                 cdraw.DrawCurvilinearGridGrayscaleCost(*(tcase.threat_record_[t_k].occupancy_grid.get()));
//         }
//     }

//     road_draw.DrawLanes(true);

//     for (auto &veh : field.GetAllVehicleEstimations())
//     {
//         if (!show_veh_id)
//             veh_draw.DrawVehicle(veh.GetFootprint());
//         else
//             veh_draw.DrawVehicle(veh.GetFootprint(), veh.id_);
//     }

//     road_draw.DrawTrafficChannelGrid(channel, false);

//     // draw state lattice
//     lattice_draw.DrawStateLattice(lattice);

//     CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
// }

// void TrafficViz::ShowLatticeInThreatField(std::vector<StateLattice> &lattice, TrafficChannel *channel, ThreatField &field, int32_t t_k, bool show_veh_id, std::string window_name, bool save_img)
// {
//     RoadMapViz &viz = RoadMapViz::GetInstance();

//     CartesianCanvas canvas = viz.CreateCanvas(true);
//     RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
//     VehicleViz veh_draw = VehicleViz(canvas);
//     CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);
//     LatticeDraw lattice_draw = LatticeDraw(canvas);
//     ThreatDraw tdraw = ThreatDraw(canvas);

//     // auto threats = field.GetAllCollisionThreats();
//     // for (auto threat : threats)
//     // {
//     //     for (auto &tcase : threat->possible_cases_)
//     //     {
//     //         if (tcase.intv_threat_record_[t_k].occupancy_grid != nullptr)
//     //             cdraw.DrawCurvilinearGridGrayscaleCost(*(tcase.threat_record_[t_k].occupancy_grid.get()));
//     //     }
//     // }

//     auto center = field.GetThreatCenter(t_k);
//     tdraw.DrawCollisionThreatField(field, t_k, center.x, center.y, 120, 50);

//     road_draw.DrawLanes(true);

//     for (auto &veh : field.GetAllVehicleEstimations())
//     {
//         if (!show_veh_id)
//             veh_draw.DrawVehicle(veh.GetFootprint());
//         else
//             veh_draw.DrawVehicle(veh.GetFootprint(), veh.id_);
//     }

//     road_draw.DrawTrafficChannelGrid(channel, false);

//     // draw state lattice
//     lattice_draw.DrawStateLattice(lattice);

//     CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
// }

// void TrafficViz::ShowTrafficChannelWithThreatField(TrafficChannel *channel, ThreatField &field, int32_t t_k, bool show_veh_id, std::string window_name, bool save_img)
// {
//     RoadMapViz &viz = RoadMapViz::GetInstance();

//     CartesianCanvas canvas = viz.CreateCanvas(true);
//     RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);
//     VehicleViz veh_draw = VehicleViz(canvas);
//     CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);
//     LatticeDraw lattice_draw = LatticeDraw(canvas);
//     ThreatDraw tdraw = ThreatDraw(canvas);

//     // auto threats = field.GetAllCollisionThreats();
//     // for (auto threat : threats)
//     // {
//     //     for (auto &tcase : threat->possible_cases_)
//     //     {
//     //         if (tcase.intv_threat_record_[t_k].occupancy_grid != nullptr)
//     //             cdraw.DrawCurvilinearGridGrayscaleCost(*(tcase.threat_record_[t_k].occupancy_grid.get()));
//     //     }
//     // }

//     auto center = field.GetThreatCenter(t_k);
//     tdraw.DrawCollisionThreatField(field, t_k, center.x, center.y, 120, 50);

//     road_draw.DrawLanes(true);

//     for (auto &veh : field.GetAllVehicleEstimations())
//     {
//         if (!show_veh_id)
//             veh_draw.DrawVehicle(veh.GetFootprint());
//         else
//             veh_draw.DrawVehicle(veh.GetFootprint(), veh.id_);
//     }

//     road_draw.DrawTrafficChannelGrid(channel, false);

//     CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
// }

// void TrafficViz::ShowPathWithThreatField(std::vector<StateLattice> &path, LookaheadZone &zone, ThreatField &field, int32_t t_k, double T, std::string window_name, bool save_img)
// {
//     RoadMapViz &viz = RoadMapViz::GetInstance();

//     std::cout << "t_k: " << t_k << std::endl;

//     CartesianCanvas canvas = viz.CreateCanvas(true);
//     RoadMapDraw road_draw = RoadMapDraw(viz.road_map_, canvas);

//     GeometryDraw gdraw(canvas);
//     LatticeDraw lattice_draw = LatticeDraw(canvas);
//     VehicleViz veh_draw = VehicleViz(canvas);
//     CurvilinearGridDraw cdraw = CurvilinearGridDraw(canvas);
//     ThreatDraw tdraw = ThreatDraw(canvas);

//     // auto center = field.GetThreatCenter(t_k);
//     // tdraw.DrawCollisionThreatField(field, t_k, center.x, center.y, 150, 120);

//     double cx = (canvas.xmax_ + canvas.xmin_) / 2.0;
//     double cy = (canvas.ymax_ + canvas.ymin_) / 2.0;
//     tdraw.DrawCollisionThreatField(field, t_k, cx, cy, canvas.xspan_, canvas.yspan_);

//     road_draw.DrawLanes(false);
//     // road_draw.DrawTrafficChannelGrid(channel, false);
//     // cdraw.DrawFilledCurvilinearGrid(zone, CvDrawColors::palegreen_color);
//     // cdraw.DrawCurvilinearGrid(zone);

//     for (auto &veh : field.GetAllVehicleEstimations())
//         veh_draw.DrawVehicle(veh.GetFootprint(), veh.id_);

//     // draw state lattice
//     lattice_draw.DrawStateLattice(path, 0.1, CvDrawColors::lime_color, 2);

//     auto dstate = zone.trajectory_.GetDesiredState(t_k * T);
//     gdraw.DrawLabelPoint(dstate.x, dstate.y, CvDrawColors::red_color, 2);

//     // draw ego vehicle
//     VehicleFP ego_veh({dstate.x, dstate.y, dstate.theta});
//     veh_draw.DrawVehicle(ego_veh.polygon, CvDrawColors::cyan_color);

//     CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
// }