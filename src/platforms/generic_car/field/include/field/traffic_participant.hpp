/* 
 * traffic_participant.hpp
 * 
 * Created on: Mar 08, 2018 15:24
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef TRAFFIC_PARTICIPANT_HPP
#define TRAFFIC_PARTICIPANT_HPP

#include <functional>

#include "field/scalar_field.hpp"

namespace librav
{
class TrafficParticipant : public ScalarField
{
  public:
    TrafficParticipant(int64_t size_x = 0, int64_t size_y = 0): ScalarField(size_x, size_y){};
    
    void SetPositionVelocity(double posx, double posy, double velx, double vely);
    void SetThreatDistribution(std::function<double(double, double)> dist_func);

    double position_x_;
    double position_y_;
    double velocity_x_;
    double velocity_y_;
};
}

#endif /* TRAFFIC_PARTICIPANT_HPP */
