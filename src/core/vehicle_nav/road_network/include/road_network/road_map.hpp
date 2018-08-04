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

#include "road_network/road_geometry.hpp"
#include "road_network/road_coordinate.hpp"

namespace librav
{
class RoadMap
{
public:
  RoadMap(std::string map_osm = "", int32_t ppm = 15);
  void LoadMapFile(std::string map_file, int32_t ppm = 15);

  RoadCoordinateFrame coordinate_;

  Eigen::MatrixXd mask_zero_;
  Eigen::MatrixXd mask_ones_;

  int32_t pixel_per_meter_ = 10;
  int32_t grid_size_x_ = 0;
  int32_t grid_size_y_ = 0;

  // sink and source lanelets in current map
  std::vector<std::string> traffic_sinks_;
  std::vector<std::string> traffic_sources_;

  bool MapReady() const { return map_loaded_; }
  void SetTrafficSinkSource(std::vector<std::string> sink, std::vector<std::string> source);

  int32_t GetLaneletIDByCenterLine(std::string cl_name);
  inline std::string GetCenterLineNameByLanelet(std::string lanelet_name) { return "cl_" + lanelet_name; }
  inline std::string GetLaneletNameFromID(int32_t id) { return ll_name_lookup_[id]; }
  inline int32_t GetLaneletIDFromName(std::string name) { return ll_id_lookup_[name]; }

  std::vector<int32_t> FindShortestRoute(std::string start_name, std::string goal_name);
  std::vector<std::string> FindShortestRouteName(std::string start_name, std::string goal_name);

  PolyLine GetLaneCenterLine(std::string lane_name);

  std::vector<int32_t> OccupiedLanelet(CartCooridnate pos);

  // dense grid for planning
  std::shared_ptr<DenseGrid> GetFullLaneBoundaryGrid();
  std::shared_ptr<DenseGrid> GetFullDrivableAreaGrid();
  std::shared_ptr<DenseGrid> GetFullCenterLineGrid();

  std::shared_ptr<DenseGrid> GetLaneBoundGrid(std::vector<std::string> lanelets);
  std::shared_ptr<DenseGrid> GetLaneBoundGrid(std::vector<int32_t> lanelets);

  std::shared_ptr<DenseGrid> GetLaneDrivableGrid(std::vector<std::string> lanelets);

private:
  bool map_loaded_ = false;

  // raw information from liblanelet
  std::unique_ptr<LLet::LaneletMap> lanelet_map_;

  std::vector<LLet::lanelet_ptr_t> lanelets_;
  std::unordered_map<std::string, int32_t> ll_id_lookup_;
  std::unordered_map<int32_t, std::string> ll_name_lookup_;

  std::unordered_map<std::string, LLet::reference_line_t> center_lines_;

  // extracted information for planning/decision making
  double x_min_ = 0.0;
  double x_max_ = 0.0;
  double y_min_ = 0.0;
  double y_max_ = 0.0;

  LLet::point_with_id_t world_origin_;
  
  std::unordered_map<int32_t, std::pair<PolyLine, PolyLine>> lane_bounds_;
  std::unordered_map<int32_t, std::shared_ptr<DenseGrid>> lane_bound_grids_;

  std::unordered_map<int32_t, std::shared_ptr<DenseGrid>> lane_drivable_grids_;

  std::unordered_map<std::string, PolyLine> lane_center_lines_;
  std::unordered_map<int32_t, std::shared_ptr<DenseGrid>> lane_centerline_grids_;

  void GenerateMapMasks();

  // ExtractXXX() to be called inside GenerateMapMasks()
  void ExtractDrivableAreas();
  void ExtractLaneCenterLines();

  std::vector<DenseGridPixel> GenerateLanePoints(const PolyLine &line);
  std::vector<DenseGridPixel> InterpolateGridPixelPoints(DenseGridPixel pt1, DenseGridPixel pt2);
};
} // namespace librav

#endif /* ROAD_MAP_HPP */
