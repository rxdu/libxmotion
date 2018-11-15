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

void ThreatEvaluation::SetEgoConfiguration(VehicleEstimation ego_est, std::shared_ptr<TrafficChannel> ego_chn, LookaheadZone ego_lookahead)
{
    ego_lookehead_ = ego_lookahead;

    field_.SetupThreatField(ego_est.GetPose(), ego_chn);
}

void ThreatEvaluation::SetTrafficConfiguration(std::vector<VehicleEstimation> ests)
{
    field_.AddVehicleEstimations(ests);
}

void ThreatEvaluation::Evaluate(int32_t step)
{
    field_.ComputeThreatField(step);
    double T = field_.GetPrecitionStepIncrement();
    std::cout << "prediction time increment: " << T << std::endl;
    for (int i = 0; i <= step; ++i)
    {
        double t = i * T;
        std::cout << "time: " << t << " , " << ego_lookehead_.trajectory_.GetDesiredState(t) << std::endl;
    }
}