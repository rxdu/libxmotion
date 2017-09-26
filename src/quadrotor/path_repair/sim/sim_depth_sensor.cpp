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
                                   fov_(M_PI / 3.0 * 2.0),
                                   ws_x_(5),
                                   ws_y_(5),
                                   ws_z_(5),
                                   unit_size_(1.0)
{
}

void SimDepthSensor::SetWorkspace(std::shared_ptr<CubeArray> ws, double side_size)
{
    // save basic parameters
    ws_x_ = ws->col_size_;
    ws_y_ = ws->row_size_;
    ws_z_ = ws->hei_size_;

    unit_size_ = side_size;

    // duplicate the workspace
    workspace_ = ws;
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
    // sph.theta = std::atan(cart.y / cart.x);
    // sph.phi = std::atan((std::sqrt(cart.x * cart.x + cart.y * cart.y) / cart.z));
    sph.theta = std::atan2(cart.y, cart.x);
    sph.phi = std::acos(cart.z / std::sqrt(cart.x * cart.x + cart.y * cart.y));

    return sph;
}

std::shared_ptr<CubeArray> SimDepthSensor::GetSensedArea(int32_t x, int32_t y, int32_t z, double yaw)
{
    // convert yaw from (-pi, pi) to (0, 2pi)
    yaw = yaw + M_PI;

    // create a cube array to represent the sensing area
    std::shared_ptr<CubeArray> carray = CubeArrayBuilder::BuildSolidCubeArray(ws_x_, ws_y_, ws_z_, unit_size_);

    // assume the cube which the sensor itself occupies is empty
    auto id = carray->GetIDFromIndex(x, y, z);
    carray->cubes_[id].occu_ = OccupancyType::FREE;

    // check cubes around the sensor
    int ext_range = range_ * 2;
    for (int k = z - ext_range; k <= z + ext_range; k++)
        for (int j = y - ext_range; j <= y + ext_range; j++)
            for (int i = x - ext_range; i <= x + ext_range; i++)
            {
                if (i >= 0 && j >= 0 && k >= 0 &&
                    i < ws_x_ && j < ws_y_ && k < ws_z_)
                {
                    auto id = carray->GetIDFromIndex(i, j, k);

                    CartesianCoordinate cart;
                    cart.x = i - x;
                    cart.y = j - y;
                    cart.z = k - z;
                    auto sph = CartesianToSpherical(cart);
                    sph.theta = sph.theta + M_PI;

                    bool theta_in_range = false;
                    bool phi_in_range = false;
                    bool r_in_range = false;

                    if (yaw <= fov_/2.0)
                    {
                        if(sph.theta >= yaw - fov_/2.0 + 2*M_PI || sph.theta <= yaw + fov_/2.0)
                            theta_in_range = true;
                    }
                    else if(yaw >= 2*M_PI - fov_/2.0)
                    {
                        if(sph.theta >= yaw - fov_/2.0 || sph.theta <= yaw + fov_/2.0 - 2*M_PI)
                            theta_in_range = true;
                    }
                    else
                    {
                        if(sph.theta >= yaw - fov_/2.0 && sph.theta <= yaw + fov_/2.0)
                            theta_in_range = true;
                    }

                    if(sph.phi <= M_PI/2.0 + fov_/2.0 || sph.phi >= M_PI/2.0 - fov_/2.0)
                        phi_in_range = true;

                    if(sph.r <= range_)
                        r_in_range = true;

                    if(theta_in_range && phi_in_range && r_in_range)
                        if (workspace_->cubes_[id].occu_ == OccupancyType::FREE)
                            carray->cubes_[id].occu_ = OccupancyType::FREE;
                }
            }

    return carray;
}