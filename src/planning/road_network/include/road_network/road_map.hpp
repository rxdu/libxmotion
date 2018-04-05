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
#include <unordered_map>

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
  RoadMap(std::string map_osm = "", int32_t ppm = 10);
  bool LoadMapFile(std::string map_file, int32_t ppm = 10);

  std::shared_ptr<DenseGrid> GetFullGrid();
  std::shared_ptr<DenseGrid> GetLaneBoundsGrid(std::vector<std::string> lanelets);
  std::shared_ptr<DenseGrid> GetLaneBoundsGrid(std::vector<int32_t> lanelets);
  std::vector<int32_t> FindShortestRoute(std::string start_name, std::string goal_name);
  std::vector<std::string> FindShortestRouteName(std::string start_name, std::string goal_name);

  std::vector<LLet::lanelet_ptr_t> OccupiedLanelet(CartCooridnate pos);

private:
  std::unique_ptr<LLet::LaneletMap> lanelet_map_;
  std::vector<LLet::lanelet_ptr_t> lanelets_;
  std::unordered_map<std::string, int32_t> ll_id_lookup_;
  std::unordered_map<int32_t, std::string> ll_name_lookup_;

  double x_min_ = 0.0;
  double x_max_ = 0.0;
  double y_min_ = 0.0;
  double y_max_ = 0.0;

  RoadCoordinateFrame coordinate_;
  std::unordered_map<int32_t, std::pair<PolyLine, PolyLine>> lane_bounds_;

  int32_t ref_lanelet_id_ = 0;
  LLet::point_with_id_t world_origin_;

  int32_t pixel_per_meter_ = 10;
  int32_t grid_size_x_ = 0;
  int32_t grid_size_y_ = 0;
  std::unordered_map<int32_t, std::shared_ptr<DenseGrid>> lane_grids_;

  void GenerateDenseGrids(int32_t pixel_per_meter);
  std::vector<DenseGridPixel> GenerateLanePoints(const PolyLine &line);
  std::vector<DenseGridPixel> InterpolateGridPixelPoints(DenseGridPixel pt1, DenseGridPixel pt2);
};
}

#endif /* ROAD_MAP_HPP */
