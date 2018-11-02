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
    VehicleEstimation() = default;
    VehicleEstimation(Pose2d _pose, double _speed);

    // Note: use getter/setter for "pose" to make sure the footprint
    //  is set properly when the pose changes
    void SetPose(Pose2d ps);
    Pose2d GetPose() const { return pose; }
    void SetPositionVariance(CovarMatrix2d covar) { pos_var = covar; }

    void SetSpeed(double spd) { speed = spd; }
    double GetSpeed() const { return speed; }
    void SetSpeedVariance(double var) { spd_var = var; }

    Polygon GetFootprint() const { return footprint.polygon; }

  private:
    Pose2d pose;
    CovarMatrix2d pos_var;

    double speed = 0;
    double spd_var = 0;
    VehicleFP footprint;
};
} // namespace librav

#endif /* VEHICLE_ESTIMATION_HPP */
