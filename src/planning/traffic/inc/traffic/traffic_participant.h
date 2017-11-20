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

#include "common/librav_common.h"

namespace librav
{

class TrafficParticipant
{
  public:
    TrafficParticipant() = default;
    ~TrafficParticipant() = default;


    virtual void Update() = 0;
};

}

#endif /* TRAFFIC_PARTICIPANT_H */
