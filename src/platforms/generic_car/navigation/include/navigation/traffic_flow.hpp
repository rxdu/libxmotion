/* 
 * traffic_flow.hpp
 * 
 * Created on: Apr 10, 2018 17:35
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAFFIC_FLOW_HPP
#define TRAFFIC_FLOW_HPP

#include <memory>
#include "road_network/road_map.hpp"

namespace librav
{
class TrafficFlow
{
  public:
    TrafficFlow() = default;

    void SetRoadMap(std::shared_ptr<RoadMap> map) { road_map_ = map; }

  private:
    std::shared_ptr<RoadMap> road_map_;
};
}

#endif /* TRAFFIC_FLOW_HPP */
