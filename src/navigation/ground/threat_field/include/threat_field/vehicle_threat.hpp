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
  public:
    struct ThreatRecord
    {
        std::shared_ptr<CurvilinearGrid> occupancy_grid;
        std::vector<VehicleStaticThreat> sub_threats;
    };

    struct ThreatCase
    {
        ThreatCase(std::shared_ptr<TrafficChannel> chn, double offset) : channel(chn),
                                                                         start_offset(offset) {}

        std::shared_ptr<TrafficChannel> channel;
        double start_offset;

        // stores all history threat information for inquiry
        std::unordered_map<int32_t, ThreatRecord> threat_record_;
        std::unordered_map<int32_t, ThreatRecord> intv_threat_record_;
    };

  public:
    VehicleThreat() = default;
    VehicleThreat(VehicleEstimation est, std::shared_ptr<TrafficMap> tmap);

    // basic threat information
    VehicleEstimation vehicle_est_;
    std::vector<std::shared_ptr<TrafficChannel>> traffic_chns_;
    std::vector<ThreatCase> possible_cases_;

    void ComputeOccupancyDistribution(int32_t k, bool calc_interval_dist = false);

    // threat value query
    double operator()(double x, double y, int32_t t_k);
    Point2d GetThreatCenter(int32_t t_k);

    double GetThreatValueAt(double x, double y, int32_t t_k) { return (*this)(x, y, t_k); }

    void PrintThreatRecordInfo();

  private:
    std::shared_ptr<TrafficMap> traffic_map_;
    DynamicThreatModel threat_model_;

    void ExtractTrafficInfo();
    void ComputeOccupancyRecord(int32_t t_k);
    void ComputeIntervalOccupancyRecord(int32_t t_k);
};
} // namespace librav

#endif /* VEHICLE_THREAT_HPP */
