/* 
 * collision_tracker.hpp
 * 
 * Created on: Aug 28, 2018 05:29
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef COLLISION_TRACKER_HPP
#define COLLISION_TRACKER_HPP

#include <memory>

#include "traffic_map/traffic_map.hpp"

namespace librav
{
class CollisionTracker
{
  public:
    CollisionTracker(std::shared_ptr<TrafficMap> map);

  private:
    std::shared_ptr<TrafficMap> traffic_map_;
};
} // namespace librav

#endif /* COLLISION_TRACKER_HPP */
