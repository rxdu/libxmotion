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
#include "traffic_map/traffic_map.hpp"

#include "cvdraw/cvdraw.hpp"

namespace ivnav
{
namespace RoadMapViz
{
void DrawLanes(CvCanvas &canvas, std::shared_ptr<RoadMap> map, bool show_center_line = true);
void DrawTrafficChannelGrid(CvCanvas &canvas, std::shared_ptr<RoadMap> map, TrafficChannel *channel, bool show_center_line = true);
void DrawTrafficChannelCenterline(CvCanvas &canvas, std::shared_ptr<RoadMap> map, TrafficChannel *channel);
}; // namespace RoadMapViz
} // namespace ivnav

#endif /* ROADMAP_DRAW_HPP */
