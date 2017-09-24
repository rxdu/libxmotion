/*
 * sim_depth_sensor.h
 *
 *  Created on: Sep 19, 2017
 *      Author: rdu
 */

#ifndef QUADROTOR_PATH_REPAIR_SIM_DEPTH_SENSOR_H
#define QUADROTOR_PATH_REPAIR_SIM_DEPTH_SENSOR_H

#include <memory>
#include <cstdint>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "planning/geometry/cube_array/cube_array.h"

namespace librav
{

struct CartesianCoordinate
{
    double x;
    double y;
    double z;
};

struct SphericalCoordinate
{
    double r;
    double theta;
    double phi;
};

class SimDepthSensor
{
public:
    SimDepthSensor();

    void SetRange(int32_t rng) { range_ = rng; };
    void SetFOV(double fov) { fov_ = fov; };
    void SetWorkspace(std::shared_ptr<CubeArray> ws, double side_size);
    void SetWorkspace(const librav_lcm_msgs::Map_t *msg, double side_size);
    std::shared_ptr<CubeArray> GetSensedArea(int32_t x, int32_t y, int32_t z, double yaw);

private:
    int32_t range_;
    double fov_;
    
    int32_t ws_x_;
    int32_t ws_y_;
    int32_t ws_z_;
    double unit_size_;

    std::shared_ptr<CubeArray> workspace_;

    SphericalCoordinate CartesianToSpherical(CartesianCoordinate& cart);
};

}

#endif /* QUADROTOR_PATH_REPAIR_SIM_DEPTH_SENSOR_H */