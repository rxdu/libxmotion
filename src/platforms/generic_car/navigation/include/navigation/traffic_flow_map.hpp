/* 
 * traffic_flow_map.hpp
 * 
 * Created on: Apr 10, 2018 17:35
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAFFIC_FLOW_MAP_HPP
#define TRAFFIC_FLOW_MAP_HPP

#include <memory>
#include <unordered_map>

#include <Eigen/Dense>

#include "road_network/road_map.hpp"
#include "navigation/map_analysis.hpp"

namespace librav
{
class TrafficFlowMap
{
public:
  // traffic flow map has to be built on top of a road map
  TrafficFlowMap() = delete;
  TrafficFlowMap(std::shared_ptr<RoadMap> map) : road_map_(map){};

  std::shared_ptr<RoadMap> road_map_;
  std::shared_ptr<RoadSquareGrid> road_grid_;

  void BuildRoadGrid(int32_t size_per_side);

  // TODO: source and sink should be detected automatically in the future
  void AddTrafficFlowSource(std::string source, GridCoordinate start);
  void AddTrafficFlowSink(std::string sink);

  void IdentifyTrafficFlow();
  std::vector<std::string> GetTrafficChannelSources() const;
  Eigen::MatrixXd GetTrafficChannelMatrix(std::string source);

  void GenerateTrafficFlowMap();
  void TraverseTrafficChannel(std::string channel);

private:
  int32_t side_length_;
  std::vector<std::string> flow_sources_;
  std::vector<std::string> flow_sinks_;
  std::unordered_map<std::string, GridCoordinate> flow_source_nodes_;

  std::unordered_map<std::string, std::set<std::string>> traffic_channels_;
  std::unordered_map<std::string, std::shared_ptr<DenseGrid>> channel_grids_;
  std::unordered_map<std::string, std::shared_ptr<RoadSquareGrid>> channel_mask_grids_;
};
}

#endif /* TRAFFIC_FLOW_MAP_HPP */
