/* 
 * road_map.hpp
 * 
 * Created on: Aug 08, 2018 22:33
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef ROAD_MAP_HPP
#define ROAD_MAP_HPP

#include <string>
#include <memory>
#include <unordered_map>

#include <liblanelet/Lanelet.hpp>
#include <liblanelet/LaneletMap.hpp>

#include "decomp/dense_grid.hpp"
#include "polygon/polygon.hpp"

#include "road_map/road_coordinate.hpp"

namespace librav
{
class RoadMap
{
public:
  RoadMap(std::string map_osm = "");
  void LoadMapFile(std::string map_file);

  // local reference frame
  RoadCoordinateFrame coordinate_;

  // sink and source lanelets in current map
  std::vector<std::string> traffic_sinks_;
  std::vector<std::string> traffic_sources_;

  bool MapReady() const { return map_loaded_; }
  void PrintInfo() const;
  void SetTrafficSinkSource(std::vector<std::string> sink, std::vector<std::string> source);

  inline std::string GetLaneletNameFromID(int32_t id) { return ll_name_lookup_[id]; }
  inline int32_t GetLaneletIDFromName(std::string name) { return ll_id_lookup_[name]; }

  // Get left and right boundary polylines by lanelet name
  std::pair<Polyline, Polyline> GetLaneBoundaryLines(std::string lane_name);
  Polygon GetLanePolygon(std::string lane_name);

  std::vector<Polyline> GetAllLaneCenterPolylines() const { return lane_center_polylines_; }
  std::vector<Polyline> GetAllLaneBoundPolylines() const { return lane_bound_polylines_; }
  std::vector<Polygon> GetAllLanePolygons() const { return lane_polygons_; }

  // Get centerline by lanelet name
  Polyline GetLaneCenterLine(std::string lane_name);

  // Find shortest route between two lanelets
  std::vector<int32_t> FindShortestRoute(std::string start_name, std::string goal_name);
  std::vector<std::string> FindShortestRouteName(std::string start_name, std::string goal_name);

  // Check occupied lanelets at given position
  std::vector<int32_t> OccupiedLanelet(CartCooridnate pos);
  void CheckLaneletCollision();

private:
  bool map_loaded_ = false;

  // basic map size information
  double x_min_ = 0.0;
  double x_max_ = 0.0;
  double y_min_ = 0.0;
  double y_max_ = 0.0;

  // origin of local reference frame
  LLet::point_with_id_t world_origin_;

  // raw information from liblanelet
  std::unique_ptr<LLet::LaneletMap> lanelet_map_;
  std::vector<LLet::lanelet_ptr_t> lanelets_;
  std::unordered_map<std::string, LLet::reference_line_t> center_lines_;

  // Lookup table for reference between lanelet name and id
  std::unordered_map<std::string, int32_t> ll_id_lookup_;
  std::unordered_map<int32_t, std::string> ll_name_lookup_;

  // Extracted information of map
  // Both lanelets and centerlines are referenced by the corresponding lanelet id
  std::unordered_map<int32_t, std::pair<Polyline, Polyline>> lane_bounds_;
  std::unordered_map<int32_t, Polyline> lane_center_lines_;

  // Polygon can be created from boundary polylines
  std::unordered_map<int32_t, Polygon> lane_polygon_;

  // for convenience
  std::vector<Polyline> lane_center_polylines_;
  std::vector<Polyline> lane_bound_polylines_;
  std::vector<Polygon> lane_polygons_;

  // Generate lane polygon from boundary lines
  void GenerateLanePolygon();

  // Mapping between lanelet and centerline
  int32_t GetLaneletIDByCenterLineName(std::string cl_name);
  inline std::string GetCenterLineNameByLanelet(std::string lanelet_name) { return "cl_" + lanelet_name; }
};
} // namespace librav

#endif /* ROAD_MAP_HPP */
