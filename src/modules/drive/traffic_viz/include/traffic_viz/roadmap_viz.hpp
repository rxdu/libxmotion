/* 
 * roadmap_viz.hpp
 * 
 * Created on: Aug 21, 2018 07:38
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef ROADMAP_VIZ_HPP
#define ROADMAP_VIZ_HPP

#include <vector>
#include <memory>
#include <cstdint>

#include "geometry/polyline.hpp"
#include "geometry/polygon.hpp"

#include "road_map/road_map.hpp"
#include "traffic_map/traffic_map.hpp"

#include "cvdraw/cvdraw.hpp"

namespace autodrive
{
namespace UGVNavViz
{
CvCanvas CreateCanvas(std::shared_ptr<RoadMap> map, int32_t ppu = 10, bool use_jetcolor = true);

void ShowLanes(std::shared_ptr<RoadMap> map, bool show_center_line = true, std::string window_name = "Lane Image", bool save_img = false);

void ShowTrafficChannel(std::shared_ptr<RoadMap> map, TrafficChannel *channel, std::string window_name = "Traffic Channel Image", bool save_img = false);
void ShowTrafficChannelCenterline(std::shared_ptr<RoadMap> map, TrafficChannel *channel, std::string window_name = "Traffic Channel Image", bool save_img = false);

void ShowVehicleOnMap(std::shared_ptr<RoadMap> map, Polygon polygon, std::string window_name = "Vehicle Image", bool save_img = false);
void ShowVehicleOnMap(std::shared_ptr<RoadMap> map, std::vector<Polygon> &polygons, std::string window_name = "Vehicle Image", bool save_img = false);
} // namespace UGVNavViz
} // namespace autodrive

#endif /* ROADMAP_VIZ_HPP */
