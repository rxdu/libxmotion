/* 
 * state_estimator.h
 * 
 * Created on: Oct 31, 2017 22:36
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include "car_state.h"

namespace librav
{

class StateEstimator
{
public:
    StateEstimator();
    ~StateEstimator() = default;

    void updateAttitude();
    void updateSpeed();
};

}

#endif /* STATE_ESTIMATOR_H */
