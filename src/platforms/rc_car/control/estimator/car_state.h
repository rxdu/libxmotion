/* 
 * car_state.h
 * 
 * Created on: Oct 31, 2017 22:32
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef CAR_STATE_H
#define CAR_STATE_H

#include <cstdint>

namespace librav
{

struct CarParam
{
    static constexpr double wheel_diameter = 0.065;
    static constexpr double gear_ratio = 6.58; // with 20T pinion gear
};

class CarState
{
public:
    CarState();
    ~CarState() = default;

public:
    float calculateSpeed(uint16_t hall_count);

private:
    double speed_;
    double yaw_;
};

}

#endif /* CAR_STATE_H */
