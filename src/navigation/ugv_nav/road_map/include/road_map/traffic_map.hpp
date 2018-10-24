/* 
 * traffic_map.hpp
 * 
 * Created on: Oct 18, 2018 08:37
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAFFIC_MAP_HPP
#define TRAFFIC_MAP_HPP

#include <vector>
#include <map>
#include <memory>

#include "decomp/curvilinear_grid.hpp"

#include "road_map/details/traffic_segment.hpp"
#include "road_map/traffic_channel.hpp"

namespace librav
{
class RoadMap;

class TrafficMap
{
public:
  TrafficMap() = delete;
  TrafficMap(RoadMap *map);

  std::vector<std::shared_ptr<TrafficChannel>> GetAllTrafficChannels();
  std::vector<std::shared_ptr<TrafficChannel>> FindTrafficChannels(SimplePoint pos);

private:
  RoadMap *road_map_;
  std::map<std::pair<std::string, std::string>, std::shared_ptr<TrafficChannel>> traffic_channels_;

  void IdentifyTrafficElements();
};
} // namespace librav

#endif /* TRAFFIC_MAP_HPP */
