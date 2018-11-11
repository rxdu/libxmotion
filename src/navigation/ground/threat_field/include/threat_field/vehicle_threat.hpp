/* 
 * vehicle_threat.hpp
 * 
 * Created on: Nov 11, 2018 09:57
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef VEHICLE_THREAT_HPP
#define VEHICLE_THREAT_HPP

#ifndef DYNAMIC_THREAT_HPP
#define DYNAMIC_THREAT_HPP

#include <cstdint>
#include <memory>
#include <vector>
#include <cmath>
#include <cassert>
#include <iostream>

#include "traffic_map/traffic_map.hpp"
#include "threat_field/dynamic_threat_model.hpp"

namespace librav
{
class VehicleThreat
{
    struct ThreatCase
    {
        ThreatCase(std::shared_ptr<TrafficChannel> chn, double offset) : channel(chn),
                                                                         start_offset(offset) {}

        std::shared_ptr<TrafficChannel> channel;
        double start_offset;
    };

  public:
    struct ThreatDist
    {
        std::shared_ptr<CurvilinearGrid> occupancy_grid;
        std::vector<VehicleStaticThreat> sub_threats;
    };

  public:
    VehicleThreat() = default;
    VehicleThreat(VehicleEstimation est, std::shared_ptr<TrafficMap> tmap);

    // basic threat information
    VehicleEstimation vehicle_est_;
    std::vector<std::shared_ptr<TrafficChannel>> traffic_chns_;

    // threat_record_ stores all history threat information
    std::unordered_map<int32_t, ThreatDist> threat_record_;
    std::unordered_map<int32_t, ThreatDist> intv_threat_record_;

    void ComputeOccupancyDistribution(int32_t k, bool calc_interval_dist = false);

    // threat value query
    double operator()(double x, double y, int32_t t_k);
    Point2d GetThreatCenter(int32_t t_k);

    ThreatDist GetThreatDistribution(int32_t t_k) { return threat_record_[t_k]; }
    ThreatDist GetIntervalThreatDistribution(int32_t t_k) { return intv_threat_record_[t_k]; }

  private:
    std::shared_ptr<TrafficMap> traffic_map_;
    DynamicThreatModel threat_model_;
    std::vector<ThreatCase> possible_cases_;

    // visualization
    bool get_interval_dist_ = false;
    int32_t vis_t_k_ = 0;

    void SetupPredictionModel();
    void ExtractTrafficInfo();
    ThreatDist GetOccupancyDistributionAt(int32_t t_k);
    ThreatDist GetIntervalOccupancyDistributionAt(int32_t t_k);
};
} // namespace librav

#endif /* DYNAMIC_THREAT_HPP */

#endif /* VEHICLE_THREAT_HPP */
