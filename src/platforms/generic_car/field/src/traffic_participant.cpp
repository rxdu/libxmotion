/* 
 * traffic_participant.cpp
 * 
 * Created on: Mar 08, 2018 15:27
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "field/traffic_participant.hpp"

using namespace librav;

void TrafficParticipant::SetPositionVelocity(double posx, double posy, double velx, double vely)
{
    position_x_ = posx;
    position_y_ = posy;

    velocity_x_ = velx;
    velocity_y_ = vely;
}

void TrafficParticipant::SetThreatDistribution(std::function<double(double, double)> dist_func)
{
    for (int64_t i = 0; i < size_x_; ++i)
        for (int64_t j = 0; j < size_y_; ++j)
            SetValueAtCoordinate(i, j, dist_func(i, j));
}