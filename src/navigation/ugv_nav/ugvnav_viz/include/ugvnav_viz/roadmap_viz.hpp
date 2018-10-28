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
#include "state_lattice/state_lattice.hpp"

#include "lightviz/details/cartesian_canvas.hpp"

namespace librav
{
class RoadMapViz
{
public:
  static void SetupRoadMapViz(std::shared_ptr<RoadMap> map, int32_t pixel_per_unit = 10);

  static void ShowLanes(bool show_center_line = true, std::string window_name = "Lane Image", bool save_img = false);

  static void ShowTrafficChannel(TrafficChannel &channel, std::string window_name = "Traffic Channel Image", bool save_img = false);
  static void ShowTrafficChannelCenterline(TrafficChannel &channel, std::string window_name = "Traffic Channel Image", bool save_img = false);

  static void ShowVehicle(Polygon &polygon, std::string window_name = "Vehicle Image", bool save_img = false);
  static void ShowVehicleFootprints(std::vector<Polygon> &polygons, std::string window_name = "Vehicle Image", bool save_img = false);

  static void ShowLatticeInTrafficChannel(std::vector<StateLattice> &lattice, TrafficChannel &channel, std::string window_name = "Traffic Channel Image", bool save_img = false);

protected:
  static RoadMapViz &GetInstance(std::shared_ptr<RoadMap> map = nullptr, int32_t pixel_per_unit = 10);

  // private constructor
  RoadMapViz(std::shared_ptr<RoadMap> map, int32_t ppu);

  std::shared_ptr<RoadMap> road_map_;
  int32_t ppu_;

  double xmin_;
  double xmax_;
  double ymin_;
  double ymax_;

  CartesianCanvas CreateCanvas();
};
} // namespace librav

#endif /* ROADMAP_VIZ_HPP */
