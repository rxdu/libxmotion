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

void ThreatEvaluation::SetTrafficConfiguration(VehicleEstimation ego_est, std::shared_ptr<TrafficChannel> ego_chn, LookaheadZone ego_lookahead, std::vector<VehicleEstimation> ests)
{
    ego_lookehead_ = ego_lookahead;
    field_.AddVehicleEstimations(ests);
    field_.SetupThreatField(ego_est.GetPose(), ego_chn);
}

void ThreatEvaluation::Evaluate(int32_t step)
{
    field_.ComputeThreatField(step);

    double T = field_.GetPrecitionStepIncrement();
    std::cout << "prediction time increment: " << T << std::endl;

    std::vector<ThreatField::ThreatComponent> threat_record;
    for (int k = 0; k <= step; ++k)
    {
        double t = k * T;
        auto dstate = ego_lookehead_.trajectory_.GetDesiredState(t);
        auto threat_comp = field_.GetThreatComponentAt(dstate.x, dstate.y, k);
        threat_record.push_back(threat_comp);

        std::cout << "time: " << t << " , " << ego_lookehead_.trajectory_.GetDesiredState(t) << std::endl;
        std::cout << "threat component: " << std::endl;
        for (auto &entry : threat_comp)
            std::cout << entry.first << " : " << entry.second << std::endl;
        std::cout << "----------------------------------------" << std::endl;
    }

    std::unordered_map<int32_t, double> total_threat;
    for (auto &entry : total_threat)
        entry.second = 0.0;
    for (auto &record : threat_record)
    {
        for (auto &entry : record)
            total_threat[entry.first] += entry.second;
    }
    std::cout << "sorted threat exposure: " << std::endl;
    for (auto &entry : total_threat)
        std::cout << entry.first << " : " << entry.second << std::endl;
}