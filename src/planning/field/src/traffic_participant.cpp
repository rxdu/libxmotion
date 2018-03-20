/* 
 * traffic_participant.cpp
 * 
 * Created on: Mar 08, 2018 15:27
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "field/traffic_participant.hpp"

#include <cmath>

using namespace librav;

template <typename DistributionT>
void TrafficParticipant::SetPositionVelocity(double posx, double posy, double velx, double vely)
{
    position_x_ = posx;
    position_y_ = posy;

    velocity_x_ = velx;
    velocity_y_ = vely;

    dist_func_.SetParameters(position_x_, position_y_, velocity_x_, velocity_y_);
    UpdateThreatDistribution();
}

template <typename DistributionT>
void TrafficParticipant::UpdateThreatDistribution()
{
    for (int64_t i = 0; i < size_x_; ++i)
        for (int64_t j = 0; j < size_y_; ++j)
            SetValueAtCoordinate(i, j, dist_func_(i, j));
}