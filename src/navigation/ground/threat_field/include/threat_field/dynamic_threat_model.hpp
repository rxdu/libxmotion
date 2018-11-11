/* 
 * dynamic_threat_model.hpp
 * 
 * Created on: Nov 02, 2018 01:37
 * Description: collision threat \psi_i for vehicle_i
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef DYNAMIC_THREAT_MODEL_HPP
#define DYNAMIC_THREAT_MODEL_HPP

#include <cstdint>
#include <memory>
#include <vector>
#include <cmath>
#include <cassert>
#include <iostream>

#include "reachability/markov_occupancy.hpp"
#include "threat_field/vehicle_estimation.hpp"
#include "threat_field/static_threat_model.hpp"

namespace librav
{
class DynamicThreatModel
{
    /************************* Parameters ****************************/
    // longitudinal: vehicle centered at cell: SStartIndex
    static constexpr double SStep = 5.0; // m
    static constexpr int32_t SSize = 20;
    static constexpr double VStep = 2.0; // m/s
    static constexpr int32_t VSize = 10;
    static constexpr int32_t SStartIndex = 3;

    // lateral: slightly wider than standard width (standard: 3.7m / 5 = 0.74)
    //          NEED to adjust LateralDistribution accordingly
    static constexpr double DeltaStep = 0.8;
    static constexpr int32_t DeltaSize = 5;
    /*****************************************************************/

    using MarkovModel = MarkovOccupancy<SSize, VSize>;

    struct LateralDistribution
    {
        static double GetProbability(int32_t delta_idx)
        {
            if (delta_idx == 0)
                return 0.35; // 0.35
            else if ((delta_idx == -1) || (delta_idx = 1))
                return 0.45 / 2.0; // 0.45
            else if ((delta_idx == -2) || (delta_idx = 2))
                return 0.1; // 0.2
            else
                return 0;
        }
    };

    friend class VehicleThreat;

//   public:
//     struct ThreatDist
//     {
//         std::shared_ptr<CurvilinearGrid> occupancy_grid;
//         std::vector<VehicleStaticThreat> sub_threats;
//     };

  public:
    DynamicThreatModel() = default;
    DynamicThreatModel(VehicleEstimation est);

    // basic threat information
    VehicleEstimation vehicle_est_;
    // std::shared_ptr<TrafficChannel> traffic_chn_;
    // std::vector<std::shared_ptr<TrafficChannel>> traffic_chns_;

    // threat_record_ stores all history threat information
    // std::unordered_map<int32_t, ThreatDist> threat_record_;
    // std::unordered_map<int32_t, ThreatDist> intv_threat_record_;

    void PrecomputeParameters(std::string file_name)
    {
        occupancy_model_->PrecomputeStateTransition(file_name);
    }

    // void ComputeOccupancyDistribution(int32_t k, bool calc_interval_dist = false);

    // threat value query
    // double operator()(double x, double y, int32_t t_k);
    // Point2d GetThreatCenter(int32_t t_k);
    // ThreatDist GetThreatDistribution(int32_t t_k) { return threat_record_[t_k]; }
    // ThreatDist GetIntervalThreatDistribution(int32_t t_k) { return intv_threat_record_[t_k]; }

    // double GetVehicleTotalThreat(double x, double y, int32_t t_k);

  private:
    // Markov model covarage: SStep * SSize = 100m, 2m/s * 10 = 20m/s
    std::shared_ptr<MarkovModel> occupancy_model_;
    double s_offset_ = 0;

    // NO NEED TO BE MODIFIED MANUALLY
    /*****************************************************************/
    const int32_t s_sidx_ = SStartIndex;
    const double s_starting_ = SStep * (s_sidx_ + 0.5);
    const double s_step_ = SStep;

    const double s_max_ = SStep * SSize;
    const double v_max_ = VStep * VSize;

    const double delta_step_ = DeltaStep;
    const int32_t delta_size_ = DeltaSize;
    /*****************************************************************/

    void SetupPredictionModel();
    // ThreatDist GetOccupancyDistributionAt(int32_t t_k);
    // ThreatDist GetIntervalOccupancyDistributionAt(int32_t t_k);
};
} // namespace librav

#endif /* DYNAMIC_THREAT_MODEL_HPP */
