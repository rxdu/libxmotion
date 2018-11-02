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

struct Pose3
{
	Point3f pos;
	EulerAngle ori;
};

struct Pose2
{
	TimeStamp t;
	Point2d position;
	double theta;
};

using Position2d = Point2d;
using Position2f = Point2f;
using Position2i = Point2i;

struct Speed
{
	Speed() : mtime(0),
			  speed(0.0){};

	Speed(int64_t time, float spd) : mtime(time),
									 speed(spd){};

	TimeStamp mtime;
	float speed;

	friend std::ostream &operator<<(std::ostream &os, const Speed &data)
	{
		os << "time_stamp: " << data.mtime << " ; speed: " << data.speed << std::endl;
		return os;
	}
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

using UAVTrajectory = std::vector<UAVTrajectoryPoint>;
} // namespace librav

#endif /* CTRL_TYPES_HPP */
