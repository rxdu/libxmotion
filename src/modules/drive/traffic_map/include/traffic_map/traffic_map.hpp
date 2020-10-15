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

#include "road_map/road_map.hpp"
#include "traffic_map/traffic_channel.hpp"

namespace ivnav
{
class RoadMap;

class TrafficMap
{
  public:
    TrafficMap() = delete;
    TrafficMap(std::shared_ptr<RoadMap> map);

    std::shared_ptr<RoadMap> road_map_;

    std::vector<std::shared_ptr<TrafficChannel>> GetAllTrafficChannels();

    std::shared_ptr<TrafficChannel> FindTrafficChannel(std::string src, std::string dst);
    std::vector<std::shared_ptr<TrafficChannel>> FindTrafficChannels(SimplePoint pos);

  private:
    std::map<std::pair<std::string, std::string>, std::shared_ptr<TrafficChannel>> traffic_channels_;

    void IdentifyTrafficElements();
};
} // namespace ivnav

#endif /* TRAFFIC_MAP_HPP */
