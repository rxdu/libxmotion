/* 
 * threat_evaluation.cpp
 * 
 * Created on: Nov 10, 2018 07:52
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "connectivity/threat_evaluation.hpp"

using namespace librav;

ThreatEvaluation::ThreatEvaluation(std::shared_ptr<RoadMap> rmap, std::shared_ptr<TrafficMap> tmap) : field_(rmap, tmap),
                                                                                                      road_map_(rmap),
                                                                                                      traffic_map_(tmap)
{
}

void ThreatEvaluation::SetTrafficConfiguration(std::vector<VehicleEstimation> ests, std::shared_ptr<TrafficChannel> ego_chn)
{
    Pose2d ego_pose(57, 36, 85.0 / 180.0 * M_PI);
    field_.AddVehicleEstimations(ests);
    field_.SetupThreatField(ego_pose, ego_chn);
}

void ThreatEvaluation::Evaluate(int32_t step)
{
    field_.ComputeThreatField(step);

    for(int i = 0; i <= step; ++i)
    {

    }
}