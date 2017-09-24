/*
 * sim_depth_sensor.cpp
 *
 *  Created on: Sep 19, 2017
 *      Author: rdu
 */

#include <cmath>

#include "quadrotor/path_repair/sim/sim_depth_sensor.h"

#include "planning/geometry/cube_array_builder.h"

using namespace librav;

SimDepthSensor::SimDepthSensor() : range_(5),
                                   fov_(M_PI / 6.0 * 4.0),
                                   ws_x_(5),
                                   ws_y_(5),
                                   ws_z_(5),
                                   unit_size_(1.0)
{
}

void SimDepthSensor::SetWorkspace(const librav_lcm_msgs::Map_t *msg, double side_size)
{
    // save basic parameters
    ws_x_ = msg->size_x;
    ws_y_ = msg->size_y;
    ws_z_ = msg->size_z;

    unit_size_ = side_size;

    // duplicate the workspace
    workspace_ = CubeArrayBuilder::BuildEmptyCubeArray(ws_x_, ws_y_, ws_z_, unit_size_);

    for (auto &v : msg->voxels)
    {
        auto id = workspace_->GetIDFromIndex(v.pos_x, v.pos_y, v.pos_z);
        if (v.occupied)
            workspace_->cubes_[id].occu_ = OccupancyType::OCCUPIED;
    }
}

SphericalCoordinate SimDepthSensor::CartesianToSpherical(CartesianCoordinate &cart)
{
    SphericalCoordinate sph;

    sph.r = std::sqrt(cart.x * cart.x + cart.y * cart.y + cart.z * cart.z);
    sph.theta = std::atan(cart.y / cart.x);
    sph.phi = std::atan((std::sqrt(cart.x * cart.x + cart.y * cart.y) / cart.z));

    return sph;
}

std::shared_ptr<CubeArray> SimDepthSensor::GetSensedArea(int32_t x, int32_t y, int32_t z, double yaw)
{
    std::shared_ptr<CubeArray> carray = CubeArrayBuilder::BuildSolidCubeArray(ws_x_, ws_y_, ws_z_, unit_size_);
}