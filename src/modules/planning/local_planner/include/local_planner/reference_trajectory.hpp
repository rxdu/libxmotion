/* 
 * reference_trajectory.hpp
 * 
 * Created on: Nov 14, 2018 03:27
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LATTICE_TRAJECTORY_HPP
#define LATTICE_TRAJECTORY_HPP

#include <vector>

#include "state_lattice/state_lattice.hpp"
#include "local_planner/speed_profile.hpp"

namespace autodrive
{
class ReferenceTrajectory
{
    struct RefPoint
    {
        RefPoint() : x(0), y(0), theta(0), kappa(0), speed(0), accel(0) {}
        RefPoint(double _x, double _y, double _theta = 0, double _kappa = 0, double _speed = 0, double _accel = 0) : x(_x),
                                                                                                                     y(_y),
                                                                                                                     theta(_theta),
                                                                                                                     kappa(_kappa),
                                                                                                                     speed(_speed),
                                                                                                                     accel(_accel) {}

        double x;
        double y;
        double theta;
        double kappa;
        double speed;
        double accel;

        friend std::ostream &operator<<(std::ostream &os, const RefPoint &state)
        {
            os << "(x,y,theta,kappa,speed): " << state.x << " , " << state.y << " , "
               << state.theta << " , " << state.kappa << " , "
               << state.speed;
            return os;
        }
    };

  public:
    ReferenceTrajectory() = default;
    ReferenceTrajectory(std::vector<StateLattice> path);

    ~ReferenceTrajectory();
    ReferenceTrajectory(const ReferenceTrajectory &other);
    ReferenceTrajectory &operator=(const ReferenceTrajectory &other);
    ReferenceTrajectory(ReferenceTrajectory &&other);
    ReferenceTrajectory &operator=(ReferenceTrajectory &&other);

    void GenerateConstantSpeedProfile(double speed);

    RefPoint GetDesiredState(double t);

    // functions that are required for creating curvilinear grid
    double GetLength() const { return total_length_; }
    void GetPositionVector(double s, double &x, double &y);
    void GetTangentVector(double s, double &x, double &y);

  private:
    std::vector<StateLattice> path_;
    double total_length_ = 0;
    SpeedProfileIF *speed_profile_ = nullptr;

    void FindPathSegment(const double s, StateLattice &lattice, double &s_offset);
    double AccumulatedDistance(double t);
};
} // namespace autodrive

#endif /* REFERENCE_TRAJECTORY_HPP */
