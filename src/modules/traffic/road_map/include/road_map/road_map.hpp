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
#include "geometry/polygon.hpp"

#include "road_map/details/road_coordinate.hpp"
#include "road_map/details/topogeo_graph.hpp"
// #include "road_map/traffic_map.hpp"

namespace ivnav
{
class RoadMap
{
  public:
    RoadMap(std::string map_osm = "");
    void LoadMapFile(std::string map_file);

    // basic map size information
    double xmin_ = 0.0;
    double xmax_ = 0.0;
    double ymin_ = 0.0;
    double ymax_ = 0.0;

    // traffic layer information
    // std::shared_ptr<TrafficMap> traffic_map_;

    // local reference frame
    RoadCoordinateFrame coordinate_;

    bool MapReady() const { return map_loaded_; }
    void PrintInfo() const;

    std::vector<std::string> GetSinks() const { return tg_graph_->sinks_; }
    std::vector<std::string> GetSources() const { return tg_graph_->sources_; }
    std::vector<std::string> GetIsolatedLanes() const { return tg_graph_->isolated_lanes_; }

    inline std::string GetLaneletNameFromID(int32_t id) { return ll_name_lookup_[id]; }
    inline int32_t GetLaneletIDFromName(std::string name) { return ll_id_lookup_[name]; }
    inline std::unordered_map<int32_t, std::string> GetLaneletIDNameMap() { return ll_name_lookup_; }

    // Get left and right boundary polylines by lanelet name
    std::pair<Polyline, Polyline> GetLaneBoundaryLines(std::string lane_name);
    Polygon GetLanePolygon(std::string lane_name);

    std::vector<Polyline> GetAllLaneCenterPolylines() const { return lane_center_polylines_; }
    std::vector<Polyline> GetAllLaneBoundPolylines() const { return lane_bound_polylines_; }
    std::vector<Polygon> GetAllLanePolygons() const { return lane_polygons_; }

    // Get centerline by lanelet name
    Polyline GetLaneCenterLine(std::string lane_name);

    // Find shortest route between two lanelets
    std::vector<int32_t> FindShortestRoute(int32_t start_id, int32_t goal_id);
    std::vector<int32_t> FindShortestRoute(std::string start_name, std::string goal_name);
    std::vector<std::string> FindShortestRouteName(int32_t start_id, int32_t goal_id);
    std::vector<std::string> FindShortestRouteName(std::string start_name, std::string goal_name);

    // Check occupied lanelets at given position
    std::vector<int32_t> FindOccupiedLanelet(CartCooridnate pos);
    std::vector<std::string> FindOccupiedLaneletNames(CartCooridnate pos);

    std::vector<std::string> FindConflictingLanes(std::vector<std::string> names) { return tg_graph_->FindConflictingLanes(names); }

    bool CheckLaneletCollision(std::string ll1, std::string ll2);
    void CheckLaneletCollision();

  private:
    bool map_loaded_ = false;

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
    std::shared_ptr<TopoGeoGraph> tg_graph_;

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
} // namespace ivnav

#endif /* ROAD_MAP_HPP */
