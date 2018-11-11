/* 
 * threat_field.hpp
 * 
 * Created on: Nov 02, 2018 06:02
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef THREAT_FIELD_HPP
#define THREAT_FIELD_HPP

#include <memory>
#include <unordered_map>

#include "traffic_map/traffic_map.hpp"
#include "threat_field/dynamic_threat_model.hpp"

namespace librav
{
class ThreatField
{
    struct ThreatRecord
    {
        ThreatRecord() = default;
        ThreatRecord(int32_t id, double c) : vehicle_id(id), cost(c) {}

        int32_t vehicle_id = -1;
        double cost = 0;
    };

  public:
    ThreatField() = delete;
    ThreatField(std::shared_ptr<RoadMap> rmap, std::shared_ptr<TrafficMap> tmap);

    void AddVehicleEstimations(std::vector<VehicleEstimation> ests);
    void SetupThreatField(std::shared_ptr<TrafficChannel> ego_chn);
    void ComputeThreatField(int32_t t_k);

    std::vector<VehicleEstimation> GetAllVehicleEstimations();
    std::vector<std::shared_ptr<DynamicThreatModel>> GetAllCollisionThreats();

    double operator()(double x, double y, int32_t t_k);
    Point2d GetThreatCenter(int32_t t_k);
    double GetCollisionThreat(double x, double y, int32_t t_k) { return (*this)(x, y, t_k); };

    // for visualization, use the above functions to query threat value
    void SetVisTimeStep(int32_t t_k) { vis_t_k_ = t_k; }
    double operator()(double x, double y);
    Point2d GetThreatCenter();

  private:
    std::shared_ptr<RoadMap> road_map_;
    std::shared_ptr<TrafficMap> traffic_map_;

    std::shared_ptr<TrafficChannel> ego_channel_;
    std::vector<std::string> conflicting_lanes_;

    std::unordered_map<int32_t, VehicleEstimation> vehicles_;
    std::unordered_map<int32_t, std::vector<std::shared_ptr<DynamicThreatModel>>> threats_;

    int32_t vis_t_k_ = 0;

    bool CheckInConflict(VehicleEstimation veh);
};
} // namespace librav

#endif /* THREAT_FIELD_HPP */
