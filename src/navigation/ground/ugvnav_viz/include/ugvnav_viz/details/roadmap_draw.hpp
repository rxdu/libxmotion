/* 
 * roadmap_draw.hpp
 * 
 * Created on: Oct 28, 2018 10:36
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef ROADMAP_DRAW_HPP
#define ROADMAP_DRAW_HPP

#include <vector>
#include <memory>
#include <cstdint>

#include "geometry/polyline.hpp"
#include "geometry/polygon.hpp"

#include "road_map/road_map.hpp"
#include "lightviz/details/cartesian_canvas.hpp"
#include "lightviz/details/geometry_draw.hpp"
#include "lightviz/details/curvilinear_grid_draw.hpp"

namespace librav
{
class RoadMapDraw
{
public:
  RoadMapDraw(std::shared_ptr<RoadMap> map, CartesianCanvas canvas);

  CartesianCanvas &GetRoadCanvas() { return canvas_; }

  void DrawLanes(bool show_center_line = true);
  void DrawTrafficChannelGrid(TrafficChannel &channel, bool show_center_line = true);
  void DrawTrafficChannelCenterline(TrafficChannel &channel);

private:
  CartesianCanvas canvas_;
  GeometryDraw gdraw_;
  CurvilinearGridDraw cdraw_;

  std::shared_ptr<RoadMap> road_map_;
  std::vector<Polyline> boundary_lines_;
  std::vector<Polyline> center_lines_;
};
} // namespace librav

#endif /* ROADMAP_DRAW_HPP */
