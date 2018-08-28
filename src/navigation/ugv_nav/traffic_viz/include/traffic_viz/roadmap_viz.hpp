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
#include "traffic_map/traffic_flow.hpp"

namespace librav
{
class RoadMapViz
{
public:
  static void SetupRoadMapViz(std::shared_ptr<RoadMap> map);

  static void ShowLanes(bool show_center_line = true, int32_t pixel_per_unit = 10, std::string window_name = "Lane Image", bool save_img = false);

  static void ShowVehicle(Polygon& polygon, int32_t pixel_per_unit = 10, std::string window_name = "Vehicle Image", bool save_img = false);
  static void ShowVehicleFootprints(std::vector<Polygon>& polygons, int32_t pixel_per_unit = 10, std::string window_name = "Vehicle Image", bool save_img = false);
  static void ShowConflictingZone(std::vector<Polygon>& highlight, std::vector<Polygon>& polygons, int32_t pixel_per_unit = 10, std::string window_name = "Vehicle Image", bool save_img = false);
  
  static void ShowLabledTrafficFlows(std::vector<TrafficFlow*>& flows, int32_t pixel_per_unit = 10, std::string window_name = "Vehicle Image", bool save_img = false);
  static void ShowLabledTrafficFlows(TrafficFlow* ego_flow, std::vector<TrafficFlow*>& flows, int32_t pixel_per_unit = 10, std::string window_name = "Vehicle Image", bool save_img = false);

private:
  static RoadMapViz &GetInstance();

  // member variables/functions
  RoadMapViz();

  double xmin_;
  double xmax_;
  double ymin_;
  double ymax_;

  std::shared_ptr<RoadMap> road_map_;
  std::vector<Polyline> boundary_lines_;
  std::vector<Polyline> center_lines_;
};
} // namespace librav

#endif /* ROADMAP_VIZ_HPP */
