/* 
 * collision_threat.hpp
 * 
 * Created on: Nov 02, 2018 01:37
 * Description: collision threat \psi_i for vehicle_i
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef COLLISION_THREAT_HPP
#define COLLISION_THREAT_HPP

#include <cstdint>
#include <memory>
#include <vector>
#include <cmath>

#include "traffic_map/traffic_map.hpp"
#include "reachability/markov_occupancy.hpp"
#include "threat_field/vehicle_estimation.hpp"
#include "threat_field/static_threat.hpp"

namespace librav
{
class CollisionThreat
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

    static void GenerateStateTransitionMatrix();

  public:
    CollisionThreat(VehicleEstimation est, std::shared_ptr<TrafficChannel> chn);

    VehicleEstimation vehicle_est_;
    std::shared_ptr<TrafficChannel> traffic_chn_;

    std::shared_ptr<CurvilinearGrid> occupancy_grid_;
    std::vector<CurvilinearCell *> nz_cells_;

    void PrecomputeParameters(std::string file_name)
    {
        occupancy_->PrecomputeStateTransition(file_name);
    }

    void UpdateOccupancyDistribution(int32_t t_k);

  private:
    // Markov model covarage: SStep * SSize = 100m, 2m/s * 10 = 20m/s
    std::unique_ptr<MarkovModel> occupancy_;
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
};
} // namespace librav

#endif /* COLLISION_THREAT_HPP */
