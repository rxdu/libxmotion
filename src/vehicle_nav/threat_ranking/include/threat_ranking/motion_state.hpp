/* 
 * motion_state.hpp
 * 
 * Created on: Aug 12, 2018 11:13
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MOTION_STATE_HPP
#define MOTION_STATE_HPP

#include <iostream>

namespace librav
{
struct MMStateEst
{
    MMStateEst() : position_x(0), position_y(0), velocity_x(0), velocity_y(0),
                   sigma_px(0), sigma_py(0), sigma_vx(0), sigma_vy(0) {}

    MMStateEst(double px, double py,
               double vx, double vy,
               double sig_px, double sig_py,
               double sig_vx, double sig_vy) : position_x(px), position_y(py), velocity_x(vx), velocity_y(vy),
                                               sigma_px(sig_px), sigma_py(sig_py), sigma_vx(sig_vx), sigma_vy(sig_vy) {}
    MMStateEst(double px, double py,
               double vx, double vy,
               double sig_p, double sig_v) : position_x(px), position_y(py), velocity_x(vx), velocity_y(vy),
                                             sigma_px(sig_p), sigma_py(sig_p), sigma_vx(sig_v), sigma_vy(sig_v) {}

    double position_x;
    double position_y;
    double velocity_x;
    double velocity_y;

    double sigma_px;
    double sigma_py;
    double sigma_vx;
    double sigma_vy;
};

//-----------------------------------------------------------------------------------//

struct MMStatePrediction
{
    MMStatePrediction(MMStateEst s, double px, double py, double vx, double vy) : base_state(s), position_x(px), position_y(py), velocity_x(vx), velocity_y(vy) {}

    MMStateEst base_state;

    double position_x;
    double position_y;
    double velocity_x;
    double velocity_y;

    bool operator==(const MMStatePrediction &other)
    {
        if (this->position_x == other.position_x &&
            this->position_y == other.position_y &&
            this->velocity_x == other.velocity_x &&
            this->velocity_y == other.velocity_y)
            return true;
        else
            return false;
    }

    bool operator!=(const MMStatePrediction &other)
    {
        if (*this == other)
            return false;
        else
            return true;
    }

    friend std::ostream &operator<<(std::ostream &os, const MMStatePrediction &data)
    {
        os << data.position_x << " , " << data.position_y << " , " << data.velocity_x << " , " << data.velocity_y;
        return os;
    }
};
} // namespace librav

#endif /* MOTION_STATE_HPP */
