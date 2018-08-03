/* 
 * motion_model.hpp
 * 
 * Created on: Aug 03, 2018 11:42
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MOTION_MODEL_HPP
#define MOTION_MODEL_HPP

#include <memory>

#include "road_network/road_map.hpp"
#include "threat_field/traffic_participant.hpp"

namespace librav
{
/// Motion model manages the evolution of detected traffic participants
class MotionModel
{
  public:
    MotionModel(std::shared_ptr<RoadMap> map);
    ~MotionModel() = default;

  private:
    std::shared_ptr<RoadMap> road_map_;
};
} // namespace librav

#endif /* MOTION_MODEL_HPP */
