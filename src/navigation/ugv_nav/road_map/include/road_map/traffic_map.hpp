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

#include "road_map/details/traffic_segment.hpp"
#include "road_map/details/traffic_channel.hpp"

namespace librav
{
class RoadMap;

class TrafficMap
{
public:
  TrafficMap() = delete;
  TrafficMap(RoadMap *map);

private:
  RoadMap *road_map_;

  void IdentifyTrafficElements();
};
} // namespace librav

#endif /* TRAFFIC_MAP_HPP */
