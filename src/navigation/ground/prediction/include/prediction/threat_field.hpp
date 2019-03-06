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
#include <vector>
#include <unordered_map>

#include "traffic_map/traffic_map.hpp"
#include "prediction/vehicle_threat.hpp"

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
    using ThreatComponent = std::unordered_map<int32_t, double>;

  public:
    ThreatField() = delete;
    ThreatField(std::shared_ptr<RoadMap> rmap, std::shared_ptr<TrafficMap> tmap);

    Pose2d ego_pose_;
    std::shared_ptr<TrafficChannel> ego_channel_;

    void AddVehicleEstimations(std::vector<VehicleEstimation> ests);
    void SetupThreatField(Pose2d ego_pose, std::shared_ptr<TrafficChannel> ego_chn);
    void ComputeThreatField(int32_t t_k);

    std::vector<VehicleEstimation> GetAllVehicleEstimations();
    std::vector<std::shared_ptr<VehicleThreat>> GetAllCollisionThreats();

    double operator()(double x, double y, int32_t t_k);
    Point2d GetThreatCenter(int32_t t_k);

    double GetCollisionThreat(double x, double y, int32_t t_k) { return (*this)(x, y, t_k); };
    ThreatComponent GetThreatComponentAt(double x, double y, int32_t t_k);

    double GetPrecitionStepIncrement() const { return step_increment_; }

  private:
    std::shared_ptr<RoadMap> road_map_;
    std::shared_ptr<TrafficMap> traffic_map_;
    double step_increment_ = 0.5;

    std::vector<std::string> conflicting_lanes_;

    std::unordered_map<int32_t, VehicleEstimation> vehicles_;
    std::unordered_map<int32_t, std::shared_ptr<VehicleThreat>> threats_;

    bool CheckInConflict(VehicleEstimation veh);
};
} // namespace librav

#endif /* THREAT_FIELD_HPP */
