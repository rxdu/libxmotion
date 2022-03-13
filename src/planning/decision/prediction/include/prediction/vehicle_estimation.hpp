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

#include "navtypes/navtypes.hpp"
#include "geometry/polygon.hpp"

namespace robosw
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

    VehicleFP(Pose2d pose)
    {
        polygon.AddPoint(2.4, -0.9);
        polygon.AddPoint(2.4, 0.9);
        polygon.AddPoint(-2.4, 0.9);
        polygon.AddPoint(-2.4, -0.9);
        polygon = polygon.TransformRT(pose.position.x, pose.position.y, pose.theta);
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
    VehicleEstimation(Pose2d _pose, CovarMatrix2d _pos_var, double _speed, double _spd_var);

    int32_t id_;

    // Note: use getter/setter for "pose" to make sure the footprint_
    //  is set properly when the pose changes
    void SetPose(Pose2d ps);
    Pose2d GetPose() const { return pose_; }
    void SetPositionVariance(CovarMatrix2d covar) { pos_var_ = covar; }
    CovarMatrix2d GetPositionVariance() const { return pos_var_; }

    void SetSpeed(double spd) { speed_ = spd; }
    double GetSpeed() const { return speed_; }
    void SetSpeedVariance(double var) { spd_var_ = var; }
    double GetSpeedVariance() const { return spd_var_; }

    Polygon GetFootprint() const { return footprint_.polygon; }

  private:
    static int32_t VehicleCount;

    Pose2d pose_;
    CovarMatrix2d pos_var_;

    double speed_ = 0;
    double spd_var_ = 0;
    VehicleFP footprint_;
};
} // namespace robosw

#endif /* VEHICLE_ESTIMATION_HPP */
