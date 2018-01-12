/* 
 * traffic_participant.h
 * 
 * Created on: Nov 20, 2017 10:31
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef TRAFFIC_PARTICIPANT_H
#define TRAFFIC_PARTICIPANT_H

#include "common/librav_types.hpp"

namespace librav
{

class TrafficParticipant
{
public:
  TrafficParticipant() = default;
  ~TrafficParticipant() = default;

  Position2Dd GetPosition() { return position_; };
  Velocity2Dd GetVelocity() { return velocity_; };

  void SetPosition(Position2Dd pos) { position_ = pos; };
  void SetVelocity(Velocity2Dd vel) { velocity_ = vel; };

  virtual void Update(double t, double dt) = 0;

protected:
  Position2Dd position_;
  Velocity2Dd velocity_;
};
}

#endif /* TRAFFIC_PARTICIPANT_H */
