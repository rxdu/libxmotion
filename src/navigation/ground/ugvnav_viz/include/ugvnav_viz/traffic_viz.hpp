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
#include "threat_field/dynamic_threat_model.hpp"
#include "threat_field/threat_field.hpp"

#include "lightviz/details/cartesian_canvas.hpp"

namespace librav
{
class TrafficViz
{
  public:
    static void SetupTrafficViz(std::shared_ptr<RoadMap> map, int32_t pixel_per_unit = 10);

    static void ShowLanes(bool show_center_line = true, std::string window_name = "Lane Image", bool save_img = false);
    static void ShowTrafficChannel(TrafficChannel &channel, std::string window_name = "Traffic Channel Image", bool save_img = false);

    static void ShowVehicle(Polygon polygon, std::string window_name = "Vehicle Image", bool save_img = false);
    static void ShowVehicle(std::vector<Polygon> &polygons, std::string window_name = "Vehicle Image", bool save_img = false);
    static void ShowVehicleInChannel(Polygon polygon, TrafficChannel &channel, std::string window_name = "Vehicle Image", bool save_img = false);

    static void ShowVehicleStaticThreat(VehicleStaticThreat threat, std::string window_name = "Collision Threat", bool save_img = false);

    static void ShowVehicleOccupancyDistribution(std::shared_ptr<DynamicThreatModel> threat, int32_t t_k, std::string window_name = "Collision Threat", bool save_img = false);
    static void ShowVehicleIntervalOccupancyDistribution(std::shared_ptr<DynamicThreatModel> threat, int32_t t_k, std::string window_name = "Collision Threat", bool save_img = false);

    static void ShowVehicleCollisionThreat(std::shared_ptr<DynamicThreatModel> threat, int32_t t_k, std::string window_name = "Collision Threat", bool save_img = false);
    static void ShowVehicleIntervalCollisionThreat(std::shared_ptr<DynamicThreatModel> threat, int32_t t_k, std::string window_name = "Collision Threat", bool save_img = false);

    static void ShowOccupancyField(ThreatField &field, int32_t t_k, bool show_veh_id = false, std::string window_name = "Occupancy Field", bool save_img = false);
    static void ShowThreatField(ThreatField &field, int32_t t_k, bool show_veh_id = false, std::string window_name = "Threat Field", bool save_img = false);

    static void ShowLatticeInTrafficChannel(std::vector<StateLattice> &lattice, TrafficChannel &channel, std::string window_name = "Traffic Channel Image", bool save_img = false);
    static void ShowLatticePathInTrafficChannel(std::vector<StateLattice> &lattice, TrafficChannel &channel, std::string window_name = "Traffic Channel Image", bool save_img = false);
    
    static void ShowLatticeWithOccupancyDistribution(std::vector<StateLattice> &lattice, TrafficChannel &channel, ThreatField &field, int32_t t_k, bool show_veh_id = false, std::string window_name = "Threat Field", bool save_img = false);
    static void ShowLatticeInThreatField(std::vector<StateLattice> &lattice, TrafficChannel &channel, ThreatField &field, int32_t t_k, bool show_veh_id = false, std::string window_name = "Threat Field", bool save_img = false);
    static void ShowTrafficChannelWithThreatField(TrafficChannel &channel, ThreatField &field, int32_t t_k, bool show_veh_id = false, std::string window_name = "Threat Field", bool save_img = false);

    // deprecated
    // static void ShowVehicleOccupancyDistribution(std::vector<std::shared_ptr<DynamicThreatModel>> threats, std::string window_name = "Collision Threat", bool save_img = false);
    // static void ShowThreatField(ThreatField &field, bool show_veh_id = false, std::string window_name = "Threat Field", bool save_img = false);
};
} // namespace librav

#endif /* TRAFFIC_VIZ_HPP */
