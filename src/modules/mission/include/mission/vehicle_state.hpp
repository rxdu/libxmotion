/* 
 * vehicle_state.hpp
 * 
 * Created on: Dec 05, 2018 22:31
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef VEHICLE_STATE_HPP
#define VEHICLE_STATE_HPP

#include <cstdint>
#include <atomic>
#include <functional>

#include <eigen3/Eigen/Dense>

#include "navtypes/navtypes.hpp"
#include "geometry/polygon.hpp"

namespace ivnav
{
struct VehicleFootprint
{
    VehicleFootprint()
    {
        polygon.AddPoint(2.4, -0.9);
        polygon.AddPoint(2.4, 0.9);
        polygon.AddPoint(-2.4, 0.9);
        polygon.AddPoint(-2.4, -0.9);
    }

    VehicleFootprint(Pose2d pose)
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

class VehicleState
{
  public:
    VehicleState() = default;
    VehicleState(int32_t id, Pose2d _pose, double _speed, CovarMatrix2d _pos_var = {}, double _spd_var = 0);

    // id is automatically managed if state is created with this constructor
    VehicleState(Pose2d _pose, double _speed, CovarMatrix2d _pos_var = {}, double _spd_var = 0);

    int32_t id_ = -1;

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
    static std::atomic<int32_t> count;

    Pose2d pose_;
    CovarMatrix2d pos_var_;

    double speed_ = 0;
    double spd_var_ = 0;

    VehicleFootprint footprint_;
};
} // namespace ivnav

#endif /* VEHICLE_STATE_HPP */
