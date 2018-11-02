/* 
 * vehicle_state.hpp
 * 
 * Created on: Nov 02, 2018 01:42
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef VEHICLE_STATE_HPP
#define VEHICLE_STATE_HPP

#include <cstdint>
#include <functional>

#include "common/librav_types.hpp"

namespace librav
{
struct VehicleState
{
    VehicleState() = default;
    VehicleState(double px, double py, double p_var, double v, double v_var) : position({px, py}), pos_var(p_var), velocity(v), vel_var(v_var) {}

    TimeStamp time_stamp = 0;

    Position2d position;
    double velocity = 0;

    double pos_var = 0;
    double vel_var = 0;

    std::function<double(double, double)> threat_func;

    double GetThreatValue(double x, double y) { return threat_func(x, y); }
};
} // namespace librav
#endif /* VEHICLE_STATE_HPP */
