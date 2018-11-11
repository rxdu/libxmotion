/* 
 * threat_evaluation.hpp
 * 
 * Created on: Nov 10, 2018 09:30
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef THREAT_EVALUATION_HPP
#define THREAT_EVALUATION_HPP

#include <memory>

#include "traffic_map/traffic_map.hpp"
#include "threat_field/threat_field.hpp"

namespace librav
{
class ThreatEvaluation
{
  public:
    ThreatEvaluation() = delete;
    ThreatEvaluation(std::shared_ptr<RoadMap> rmap, std::shared_ptr<TrafficMap> tmap);

    ThreatField field_;

    void SetTrafficConfiguration(std::vector<VehicleEstimation> ests, std::shared_ptr<TrafficChannel> ego_chn);
    void Evaluate(int32_t step);

  private:
    std::shared_ptr<RoadMap> road_map_;
    std::shared_ptr<TrafficMap> traffic_map_;
};
} // namespace librav

#endif /* THREAT_EVALUATION_HPP */
