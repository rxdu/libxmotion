/* 
 * traffic_viz.hpp
 * 
 * Created on: Nov 02, 2018 09:00
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAFFIC_VIZ_HPP
#define TRAFFIC_VIZ_HPP

#include <vector>
#include <memory>
#include <cstdint>

#include "geometry/polyline.hpp"
#include "geometry/polygon.hpp"

#include "road_map/road_map.hpp"
#include "traffic_map/traffic_map.hpp"
#include "state_lattice/state_lattice.hpp"
#include "threat_field/vehicle_threat.hpp"
#include "threat_field/threat_field.hpp"
#include "local_planner/lattice_graph.hpp"
#include "local_planner/reference_trajectory.hpp"
#include "local_planner/lookahead_zone.hpp"

#include "cvdraw/cvdraw.hpp"

namespace librav
{
namespace UGVNavViz
{
void ShowVehicleInChannel(std::shared_ptr<RoadMap> map, Polygon polygon, TrafficChannel *channel, std::string window_name = "Traffic Channel", bool save_img = false);

void ShowVehicleStaticThreat(std::shared_ptr<RoadMap> map, VehicleStaticThreat threat, std::string window_name = "Collision Threat", bool save_img = false);

void ShowVehicleOccupancyDistribution(std::shared_ptr<RoadMap> map, std::shared_ptr<VehicleThreat> threat, int32_t t_k, std::string window_name = "Collision Threat", bool save_img = false);
void ShowVehicleCollisionThreat(std::shared_ptr<RoadMap> map, std::shared_ptr<VehicleThreat> threat, int32_t t_k, std::string window_name = "Collision Threat", bool save_img = false);

void ShowLatticeInTrafficChannel(std::shared_ptr<RoadMap> map, std::vector<StateLattice> &lattice, TrafficChannel *channel, std::string window_name = "Traffic Channel Image", bool save_img = false);
void ShowLatticePathInTrafficChannel(std::shared_ptr<RoadMap> map, std::vector<StateLattice> &path, TrafficChannel *channel, std::string window_name = "Traffic Channel Image", bool save_img = false);
void ShowLatticePathInTrafficChannel(std::shared_ptr<RoadMap> map, std::shared_ptr<Graph<LatticeGraph::LatticeNode, StateLattice>> graph, std::vector<StateLattice> &path, TrafficChannel *channel, std::string window_name = "Traffic Channel Image", bool save_img = false);

void ShowLatticePathWithLookaheadZone(std::shared_ptr<RoadMap> map, std::vector<StateLattice> &path, LookaheadZone &zone, TrafficChannel *channel, std::string window_name = "Traffic Channel Image", bool save_img = false);

void ShowOccupancyField(std::shared_ptr<RoadMap> map, ThreatField &field, int32_t t_k, bool show_veh_id = false, std::string window_name = "Occupancy Field", bool save_img = false);
void ShowThreatField(std::shared_ptr<RoadMap> map, ThreatField &field, int32_t t_k, bool show_veh_id = false, std::string window_name = "Threat Field", bool save_img = false);

void ShowLatticeWithOccupancyDistribution(std::shared_ptr<RoadMap> map, std::vector<StateLattice> &lattice, TrafficChannel *channel, ThreatField &field, int32_t t_k, bool show_veh_id = false, std::string window_name = "Threat Field", bool save_img = false);
void ShowLatticeInThreatField(std::shared_ptr<RoadMap> map, std::vector<StateLattice> &lattice, TrafficChannel *channel, ThreatField &field, int32_t t_k, bool show_veh_id = false, std::string window_name = "Threat Field", bool save_img = false);
void ShowTrafficChannelWithThreatField(std::shared_ptr<RoadMap> map, TrafficChannel *channel, ThreatField &field, int32_t t_k, bool show_veh_id = false, std::string window_name = "Threat Field", bool save_img = false);
void ShowPathWithThreatField(std::shared_ptr<RoadMap> map, std::vector<StateLattice> &path, LookaheadZone &zone, ThreatField &field, int32_t t_k, double T = 0.5, std::string window_name = "Traffic Channel Image", bool save_img = false);
}; // namespace UGVNavViz
} // namespace librav

#endif /* TRAFFIC_VIZ_HPP */
