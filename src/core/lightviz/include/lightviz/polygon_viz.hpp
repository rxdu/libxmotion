/* 
 * polygon_viz.hpp
 * 
 * Created on: Aug 10, 2018 09:11
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef POLYGON_VIZ_HPP
#define POLYGON_VIZ_HPP

#include <vector>

#include "geometry/polygon.hpp"

namespace librav
{
namespace LightViz
{
void ShowPolyline(const Polyline &polyline, int32_t pixel_per_unit = 10, std::string window_name = "Polyline Image", bool save_img = false);
void ShowPolyline(const std::vector<Polyline> &polylines, int32_t pixel_per_unit = 10, std::string window_name = "Polyline Image", bool save_img = false);

void ShowPolygon(const Polygon &polygon, int32_t pixel_per_unit = 10, std::string window_name = "Polygon Image", bool save_img = false);
void ShowPolygon(const std::vector<Polygon> &polygons, int32_t pixel_per_unit = 10, std::string window_name = "Polygon Image", bool save_img = false);

void ShowLanePolylines(const std::vector<Polyline> &bounds, const std::vector<Polyline> &centers, int32_t pixel_per_unit = 10, std::string window_name = "Lane Polyline Image", bool save_img = false);
void ShowPathInLane(const std::vector<Polyline> &bounds, const std::vector<Polyline> &centers, std::vector<Polyline> &path, int32_t pixel_per_unit = 10, std::string window_name = "Lane Polyline Image", bool save_img = false);
} // namespace LightViz
} // namespace librav

#endif /* POLYGON_VIZ_HPP */
