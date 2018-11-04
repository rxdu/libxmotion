/* 
 * vehicle_estimation.hpp
 * 
 * Created on: Nov 02, 2018 01:42
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef VEHICLE_ESTIMATION_HPP
#define VEHICLE_ESTIMATION_HPP

#include <cstdint>
#include <functional>

#include <eigen3/Eigen/Dense>

#include "common/librav_types.hpp"
#include "geometry/polygon.hpp"
#include "traffic_map/traffic_channel.hpp"

namespace librav
{
struct VehicleFP
{
    VehicleFP()
    {
        polygon.AddPoint(2.4, -0.9);
        polygon.AddPoint(2.4, 0.9);
        polygon.AddPoint(-2.4, 0.9);
        polygon.AddPoint(-2.4, -0.9);
    }

    void TransformRT(double dx, double dy, double dtheta)
    {
        polygon = polygon.TransformRT(dx, dy, dtheta);
    }

    Polygon polygon;
};

class VehicleEstimation
{
  public:
    VehicleEstimation();
    VehicleEstimation(Pose2d _pose, double _speed);
    VehicleEstimation(Pose2d _pose, double _speed, std::shared_ptr<TrafficChannel> chn);
    VehicleEstimation(Pose2d _pose, double _speed, std::vector<std::shared_ptr<TrafficChannel>> chns);

    int32_t id_;

    // Note: use getter/setter for "pose" to make sure the footprint
    //  is set properly when the pose changes
    void SetPose(Pose2d ps);
    Pose2d GetPose() const { return pose; }
    void SetPositionVariance(CovarMatrix2d covar) { pos_var = covar; }
    CovarMatrix2d GetPositionVariance() const { return pos_var; }

    void SetSpeed(double spd) { speed = spd; }
    double GetSpeed() const { return speed; }
    void SetSpeedVariance(double var) { spd_var = var; }
    double GetSpeedVariance() const { return spd_var; }

    void SetOccupiedChannels(std::vector<std::shared_ptr<TrafficChannel>> chns) { occupied_channels_ = chns; }
    std::vector<std::shared_ptr<TrafficChannel>> GetOccupiedChannels() { return occupied_channels_; }

    Polygon GetFootprint() const { return footprint.polygon; }

  private:
    static int32_t VehicleCount;
    std::vector<std::shared_ptr<TrafficChannel>> occupied_channels_;

    Pose2d pose;
    CovarMatrix2d pos_var;

    double speed = 0;
    double spd_var = 0;
    VehicleFP footprint;
};
} // namespace librav

#endif /* VEHICLE_ESTIMATION_HPP */
