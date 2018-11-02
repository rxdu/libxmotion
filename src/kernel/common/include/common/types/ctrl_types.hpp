/* 
 * ctrl_types.hpp
 * 
 * Created on: Oct 10, 2018 11:50
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CTRL_TYPES_HPP
#define CTRL_TYPES_HPP

#include <cstdint>
#include <vector>
#include <iostream>

#include "common/types/base_types.hpp"

namespace librav
{
struct EulerAngle
{
	float roll;
	float pitch;
	float yaw;
};

struct Quaternion
{
	float x;
	float y;
	float z;
	float w;
};

struct Pose
{
	Point3f pos;
	EulerAngle ori;
};

struct UAVTrajectoryPoint
{
	bool point_empty;
	float positions[3];
	float velocities[3];
	float accelerations[3];
	float jerks[3];
	float yaw;
	float yaw_rate;
	uint64_t duration; // in milliseconds
};

typedef std::vector<UAVTrajectoryPoint> UAVTrajectory;
} // namespace librav

#endif /* CTRL_TYPES_HPP */
