/* 
 * road_map.hpp
 * 
 * Created on: Apr 03, 2018 14:17
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef ROAD_MAP_HPP
#define ROAD_MAP_HPP

#include <string>
#include <memory>

#include "decomp/dense_grid.hpp"

#include <liblanelet/Lanelet.hpp>
#include <liblanelet/LaneletMap.hpp>

#include "road_network/polyline.hpp"
#include "road_network/road_coordinate.hpp"

namespace librav
{
class RoadMap
{
public:
  RoadMap(std::string map_osm);
  void LoadMapFile(std::string map_file = "");

  std::unique_ptr<DenseGrid> dense_grid_;

  void GenerateDenseGrid(int32_t pixel_per_meter);

private:
  std::unique_ptr<LLet::LaneletMap> lanelet_map_;
  std::vector<LLet::lanelet_ptr_t> lanelets_;

  double x_min_ = 0.0;
  double x_max_ = 0.0;
  double y_min_ = 0.0;
  double y_max_ = 0.0;

  RoadCoordinateFrame coordinate_;
  std::unordered_map<int32_t, std::pair<PolyLine, PolyLine>> lane_bounds_;

  const int32_t ref_lanelet_id_ = -1;
  LLet::point_with_id_t world_origin_;

  std::vector<DenseGridPixel> GenerateLanePoints(const PolyLine &line, double resolution = 0.01);
  std::vector<DenseGridPixel> InterpolateGridPixelPoints(PolyLinePoint plt1, PolyLinePoint plt2, double resolution);
  std::vector<DenseGridPixel> InterpolateGridPixelPoints(DenseGridPixel pt1, DenseGridPixel pt2);
};
}

#endif /* ROAD_MAP_HPP */
